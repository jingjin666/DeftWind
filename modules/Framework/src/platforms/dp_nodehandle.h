/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_nodehandle.h
 *
 * DP Middleware Wrapper Node Handle
 */
#pragma once

/* includes for all platforms */
#include "dp_subscriber.h"
#include "dp_publisher.h"
#include "dp_middleware.h"
#include "dp_app.h"

/* includes when building for NuttX */
#include <dp_posix.h>
#include <poll.h>

#include <functional>

namespace dp
{

//Building for NuttX
class __EXPORT NodeHandle
{
public:
	NodeHandle(AppState &a) :
		_subs(),
		_pubs(),
		_sub_min_interval(nullptr),
		_appState(a)
	{}

	~NodeHandle()
	{
		/* Empty subscriptions list */
		SubscriberNode *sub = _subs.getHead();
		int count = 0;

		while (sub != nullptr) {
			if (count++ > kMaxSubscriptions) {
				DP_WARN("exceeded max subscriptions");
				break;
			}

			SubscriberNode *sib = sub->getSibling();
			delete sub;
			sub = sib;
		}

		/* Empty publications list */
		PublisherNode *pub = _pubs.getHead();
		count = 0;

		while (pub != nullptr) {
			if (count++ > kMaxPublications) {
				DP_WARN("exceeded max publications");
				break;
			}

			PublisherNode *sib = pub->getSibling();
			delete pub;
			pub = sib;
		}
	};

	/**
	 * Subscribe with callback to function
	 * @param fp		Callback, executed on receiving a new message
	 * @param interval	Minimal interval between calls to callback
	 */

	template<typename T>
	Subscriber<T> *subscribe(void(*fp)(const T &),  unsigned interval)
	{
		(void)interval;
		SubscriberUORBCallback<T> *sub_dp = new SubscriberUORBCallback<T>(interval, std::bind(fp, std::placeholders::_1));
		update_sub_min_interval(interval, sub_dp);
		_subs.add((SubscriberNode *)sub_dp);
		return (Subscriber<T> *)sub_dp;
	}

	/**
	 * Subscribe with callback to class method
	 * @param fb		Callback, executed on receiving a new message
	 * @param obj	        pointer class instance
	 */
	template<typename T, typename C>
	Subscriber<T> *subscribe(void(C::*fp)(const T &), C *obj, unsigned interval)
	{
		(void)interval;
		SubscriberUORBCallback<T> *sub_dp = new SubscriberUORBCallback<T>(interval, std::bind(fp, obj, std::placeholders::_1));
		update_sub_min_interval(interval, sub_dp);
		_subs.add((SubscriberNode *)sub_dp);
		return (Subscriber<T> *)sub_dp;
	}

	/**
	 * Subscribe without callback to function
	 * @param interval	Minimal interval between data fetches from orb
	 */

	template<typename T>
	Subscriber<T> *subscribe(unsigned interval)
	{
		(void)interval;
		SubscriberUORB<T> *sub_dp = new SubscriberUORB<T>(interval);
		update_sub_min_interval(interval, sub_dp);
		_subs.add((SubscriberNode *)sub_dp);
		return (Subscriber<T> *)sub_dp;
	}

	/**
	 * Advertise topic
	 */
	template<typename T>
	Publisher<T> *advertise()
	{
		PublisherUORB<T> *pub = new PublisherUORB<T>();
		_pubs.add(pub);
		return (Publisher<T> *)pub;
	}

	/**
	 * Calls all callback waiting to be called
	 */
	void spinOnce()
	{
		/* Loop through subscriptions, call callback for updated subscriptions */
		SubscriberNode *sub = _subs.getHead();
		int count = 0;

		while (sub != nullptr) {
			if (count++ > kMaxSubscriptions) {
				DP_WARN("exceeded max subscriptions");
				break;
			}

			sub->update();
			sub = sub->getSibling();
		}
	}

	/**
	 * Keeps calling callbacks for incoming messages, returns when module is terminated
	 */
	void spin()
	{
		while (!_appState.exitRequested()) {
			const int timeout_ms = 100;

			/* Only continue in the loop if the nodehandle has subscriptions */
			if (_sub_min_interval == nullptr) {
				usleep(timeout_ms * 1000);
				continue;
			}

			/* Poll fd with smallest interval */
			dp_pollfd_struct_t pfd;
			pfd.fd = _sub_min_interval->getUORBHandle();
			pfd.events = POLLIN;
			dp_poll(&pfd, 1, timeout_ms);
			spinOnce();
		}
	}
protected:
	static const uint16_t kMaxSubscriptions = 100;
	static const uint16_t kMaxPublications = 100;
	List<SubscriberNode *> _subs;		/**< Subcriptions of node */
	List<PublisherNode *> _pubs;		/**< Publications of node */
	SubscriberNode *_sub_min_interval;	/**< Points to the sub with the smallest interval
							  of all Subscriptions in _subs*/

	AppState	&_appState;

	/**
	 * Check if this is the smallest interval so far and update _sub_min_interval
	 */
	template<typename T>
	void update_sub_min_interval(unsigned interval, SubscriberUORB<T> *sub)
	{
		if (_sub_min_interval == nullptr || _sub_min_interval->get_interval() > interval) {
			_sub_min_interval = sub;
		}
	}
};
}
