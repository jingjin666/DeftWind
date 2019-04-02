/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_subscriber.h
 *
 * DP Subscriber API, implements subscribing to messages from a nodehandle
 */
#pragma once

#ifndef CONFIG_ARCH_BOARD_SIM
#include <functional>
#include <type_traits>
#endif

/* includes when building for NuttX */
#include <uORB/Subscription.hpp>
#include <containers/List.hpp>
#include "dp_nodehandle.h"

namespace dp
{

/**
 * Untemplated subscriber base class
 * */
class __EXPORT SubscriberBase
{
public:
	SubscriberBase() {};
	virtual ~SubscriberBase() {};

};

/**
 * Subscriber class which is used by nodehandle
 */
template<typename T>
class __EXPORT Subscriber :
	public SubscriberBase
{
public:
	Subscriber() :
		SubscriberBase(),
		_msg_current()
	{};

	virtual ~Subscriber() {}

	/* Accessors*/
	/**
	 * Get the last message value
	 */
	virtual T &get() {return _msg_current;}

	/**
	 * Get the last native message value
	 */
	virtual decltype(((T *)nullptr)->data()) data()
	{
		return _msg_current.data();
	}

protected:
	T _msg_current;				/**< Current Message value */
};

// Building for NuttX
/**
 * Because we maintain a list of subscribers we need a node class
 */
class __EXPORT SubscriberNode :
	public ListNode<SubscriberNode *>
{
public:
	SubscriberNode(unsigned interval) :
		ListNode(),
		_interval(interval)
	{}

	virtual ~SubscriberNode() {}

	virtual void update() = 0;

	virtual int getUORBHandle() = 0;

	unsigned get_interval() { return _interval; }

protected:
	unsigned _interval;

};

/**
 * Subscriber class that is templated with the uorb subscription message type
 */
template<typename T>
class __EXPORT SubscriberUORB :
	public Subscriber<T>,
	public SubscriberNode
{
public:

	/**
	 * Construct SubscriberUORB by providing orb meta data without callback
	 * @param interval	Minimal interval between calls to callback
	 */
	SubscriberUORB(unsigned interval) :
		SubscriberNode(interval),
		_uorb_sub(new uORB::SubscriptionBase(T::handle(), interval))
	{}

	virtual ~SubscriberUORB()
	{
		delete _uorb_sub;
	};

	/**
	 * Update Subscription
	 * Invoked by the list traversal in NodeHandle::spinOnce
	 */
	virtual void update()
	{
		if (!_uorb_sub->updated()) {
			/* Topic not updated, do not call callback */
			return;
		}

		_uorb_sub->update(get_void_ptr());
	};

	/* Accessors*/
	int getUORBHandle() { return _uorb_sub->getHandle(); }

protected:
	uORB::SubscriptionBase *_uorb_sub;	/**< Handle to the subscription */

#ifndef CONFIG_ARCH_BOARD_SIM
	typename std::remove_reference < decltype(((T *)nullptr)->data()) >::type getUORBData()
	{
		return (typename std::remove_reference < decltype(((T *)nullptr)->data()) >::type) * _uorb_sub;
	}
#endif

	/**
	 * Get void pointer to last message value
	 */
	void *get_void_ptr() { return (void *) & (this->_msg_current.data()); }

};

//XXX reduce to one class with overloaded constructor?
template<typename T>
class __EXPORT SubscriberUORBCallback :
	public SubscriberUORB<T>
{
public:
	/**
	 * Construct SubscriberUORBCallback by providing orb meta data
	 * @param cbf		Callback, executed on receiving a new message
	 * @param interval	Minimal interval between calls to callback
	 */
	SubscriberUORBCallback(unsigned interval
#ifndef CONFIG_ARCH_BOARD_SIM
			       , std::function<void(const T &)> cbf)
#else
			      )
#endif
		:
		SubscriberUORB<T>(interval),
		_cbf(cbf)
	{}

	virtual ~SubscriberUORBCallback() {};

	/**
	 * Update Subscription
	 * Invoked by the list traversal in NodeHandle::spinOnce
	 * If new data is available the callback is called
	 */
	virtual void update()
	{
		if (!this->_uorb_sub->updated()) {
			/* Topic not updated, do not call callback */
			return;
		}

		/* get latest data */
		this->_uorb_sub->update(this->get_void_ptr());


		/* Check if there is a callback */
		if (_cbf == nullptr) {
			return;
		}

		/* Call callback which performs actions based on this data */
		_cbf(Subscriber<T>::get());

	};

protected:
#ifndef CONFIG_ARCH_BOARD_SIM
	std::function<void(const T &)> _cbf;	/**< Callback that the user provided on the subscription */
#endif
};

}
