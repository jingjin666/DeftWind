#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_UAVRS

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>
#include <nuttx/progmem.h>

#include <AP_BoardConfig/AP_BoardConfig.h>

#include "Storage.h"
using namespace UAVRS;

/*
  This stores eeprom data in the UAVRS MTD interface with a 4k size, and
  a in-memory buffer. This keeps the latency and devices IOs down.
*/

// name the storage file after the sketch so you can use the same sd
// card for ArduCopter and ArduPlane
#define STORAGE_DIR "/fs/microsd/APM"
//#define SAVE_STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".sav"
#define MTD_PARAMS_FILE "/fs/mtd"

extern const AP_HAL::HAL& hal;

extern "C" int mtd_main(int, char **);

Storage::Storage(void) :
    _perf_storage(perf_alloc(PC_ELAPSED, "APM_storage")),
    _perf_errors(perf_alloc(PC_COUNT, "APM_storage_errors"))
{
}

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }

    _dirty_mask.clearall();

    // load from storage backend
#if USE_FLASH_STORAGE
    _flash_load();
#else
    _mtd_load();
#endif

#ifdef SAVE_STORAGE_FILE
    fd = open(SAVE_STORAGE_FILE, O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (fd != -1) {
        write(fd, _buffer, sizeof(_buffer));
        close(fd);
        ::printf("Saved storage file %s\n", SAVE_STORAGE_FILE);
    }
#endif
    _initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
*/
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    uint16_t end = loc + length;
    for (uint16_t line=loc>>UAVRS_STORAGE_LINE_SHIFT;
         line <= end>>UAVRS_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised || _dirty_mask.empty()) {
        return;
    }
    perf_begin(_perf_storage);

#if !USE_FLASH_STORAGE
    if (_fd == -1) {
        _fd = open(MTD_PARAMS_FILE, O_WRONLY);
        if (_fd == -1) {
            perf_end(_perf_storage);
            perf_count(_perf_errors);
            return;
        }
    }
#endif

    // write out the first dirty line. We don't write more
    // than one to keep the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<UAVRS_STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == UAVRS_STORAGE_NUM_LINES) {
        // this shouldn't be possible
        perf_end(_perf_storage);
        perf_count(_perf_errors);
        return;
    }

    // save to storage backend
#if USE_FLASH_STORAGE
    _flash_write(i);
#else
    _mtd_write(i);
#endif
    
    perf_end(_perf_storage);
}

#if !USE_FLASH_STORAGE
void Storage::bus_lock(bool lock)
{
}

/*
  write one storage line. This also updates _dirty_mask. 
*/
void Storage::_mtd_write(uint16_t line)
{
    uint16_t ofs = line * UAVRS_STORAGE_LINE_SIZE;
    if (lseek(_fd, ofs, SEEK_SET) != ofs) {
        return;
    }

    // mark the line clean
    _dirty_mask.clear(line);
    
    bus_lock(true);
    ssize_t ret = write(_fd, &_buffer[ofs], UAVRS_STORAGE_LINE_SIZE);
    bus_lock(false);

    if (ret != UAVRS_STORAGE_LINE_SIZE) {
        // write error - likely EINTR
        _dirty_mask.set(line);
        close(_fd);
        _fd = -1;
        perf_count(_perf_errors);
    }
}

/*
  load all data from mtd
 */
void Storage::_mtd_load(void)
{
    if (AP_BoardConfig::px4_start_driver(mtd_main, "mtd", "start " MTD_PARAMS_FILE)) {
        printf("mtd: started OK\n");
        #if 0
        if (AP_BoardConfig::px4_start_driver(mtd_main, "mtd", "readtest " MTD_PARAMS_FILE)) {
            printf("mtd: readtest OK\n");
        } else {
            AP_BoardConfig::sensor_config_error("mtd: failed readtest");
        }
        #endif
    } else {
        AP_BoardConfig::sensor_config_error("mtd: failed start");
    }

    int fd = open(MTD_PARAMS_FILE, O_RDONLY);
    if (fd == -1) {
        AP_HAL::panic("Failed to open " MTD_PARAMS_FILE);
    }
    const uint16_t chunk_size = 128;
    for (uint16_t ofs=0; ofs<sizeof(_buffer); ofs += chunk_size) {
        bus_lock(true);
        ssize_t ret = read(fd, &_buffer[ofs], chunk_size);
        bus_lock(false);
        if (ret != chunk_size) {
            ::printf("storage read of %u bytes at %u to %p failed - got %d errno=%d\n",
                     (unsigned)sizeof(_buffer), (unsigned)ofs, &_buffer[ofs], (int)ret, (int)errno);
            AP_HAL::panic("Failed to read " MTD_PARAMS_FILE);
        }
    }
    close(fd);
}

#else // USE_FLASH_STORAGE

/*
  load all data from flash
 */
void Storage::_flash_load(void)
{
    _flash_page = up_progmem_npages() - 2;
    if (up_progmem_pagesize(_flash_page) != 128*1024U ||
        up_progmem_pagesize(_flash_page+1) != 128*1024U) {
        AP_HAL::panic("Bad flash page sizes %u %u",
                      up_progmem_pagesize(_flash_page),
                      up_progmem_pagesize(_flash_page+1));
    }
        
    printf("Storage: Using flash pages %u and %u\n", _flash_page, _flash_page+1);
    
    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
}

/*
  write one storage line. This also updates _dirty_mask. 
*/
void Storage::_flash_write(uint16_t line)
{
    if (_flash.write(line*UAVRS_STORAGE_LINE_SIZE, UAVRS_STORAGE_LINE_SIZE)) {
        // mark the line clean
        _dirty_mask.clear(line);
    } else {
        perf_count(_perf_errors);
    }
}

/*
  callback to write data to flash
 */
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    size_t base_address = up_progmem_getaddress(_flash_page+sector);
    return up_progmem_write(base_address+offset, data, length) == length;
}

/*
  callback to read data from flash
 */
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    size_t base_address = up_progmem_getaddress(_flash_page+sector);
    const uint8_t *b = ((const uint8_t *)base_address)+offset;
    memcpy(data, b, length);
    return true;
}

/*
  callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
    return up_progmem_erasepage(_flash_page+sector) > 0;
}

/*
  callback to check if erase is allowed
 */
bool Storage::_flash_erase_ok(void)
{
    // only allow erase while disarmed
    return !hal.util->get_soft_armed();
}
#endif // USE_FLASH_STORAGE


#endif // CONFIG_HAL_BOARD
