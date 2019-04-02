/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file dp_publisher.h
 *
 * DP Publisher API, implements publishing of messages from a nodehandle
 */
#pragma once

/* includes when building for NuttX */
#include <uORB/Publication.hpp>
#include <containers/List.hpp>

#include <platforms/dp_message.h>

namespace dp
{

/**
 * Untemplated publisher base class
 * */
class __EXPORT PublisherBase
{
public:
	PublisherBase() {};
	~PublisherBase() {};
};

/**
 * Publisher base class, templated with the message type
 * */
template <typename T>
class __EXPORT Publisher
{
public:
	Publisher() {};
	~Publisher() {};

	virtual int publish(const T &msg)  = 0;
};

/**
 * Because we maintain a list of publishers we need a node class
 */
class __EXPORT PublisherNode :
	public ListNode<PublisherNode *>
{
public:
	PublisherNode() :
		ListNode()
	{}

	virtual ~PublisherNode() {}

	virtual void update() = 0;
};

template <typename T>
class __EXPORT PublisherUORB :
	public Publisher<T>,
	public PublisherNode

{
public:
	/**
	 * Construct Publisher by providing orb meta data
	 */
	PublisherUORB() :
		Publisher<T>(),
		PublisherNode(),
		_uorb_pub(new uORB::PublicationBase(T::handle()))
	{}

	~PublisherUORB()
	{
		delete _uorb_pub;
	};

	/** Publishes msg
	 * @param msg	    the message which is published to the topic
	 */
	int publish(const T &msg)
	{
		_uorb_pub->update((void *) & (msg.data()));
		return 0;
	}

	/**
	 * Empty callback for list traversal
	 */
	void update() {} ;
private:
	uORB::PublicationBase *_uorb_pub;	/**< Handle to the publisher */

};
}
