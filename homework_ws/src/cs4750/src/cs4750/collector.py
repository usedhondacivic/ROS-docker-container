"""ROS message collector."""

import message_filters
import rospy

try:
    import queue
except ImportError:
    import Queue as queue  # noqa


class MessageCollector:
    """Collect all messages published to a ROS topic."""

    def __init__(self, topic, msg_type, queue_size=10):
        self._topic = topic
        self._msg_type = msg_type
        self._queue_size = queue_size

    def start(self, duration=None, max_msgs=-1):
        self.msgs = queue.Queue(maxsize=max_msgs)
        self.__cmd_sub = rospy.Subscriber(
            self._topic, self._msg_type, self._msg_callback, queue_size=self._queue_size
        )
        if duration:
            rospy.sleep(duration)
            return self.stop()

    def stop(self):
        self.__cmd_sub.unregister()
        self.__cmd_sub = None
        return list(self.msgs.queue)

    def _msg_callback(self, msg):
        self.msgs.put(msg)


class SynchronizedMessageCollector:
    """Collect approximately synchronized messages published to a set of ROS topics."""

    def __init__(self, topics, msg_types, queue_size=10, sync_tolerance=0.1):
        self._topics = topics
        self._msg_types = msg_types
        self._queue_size = queue_size
        self._sync_tolerance = sync_tolerance

    def start(self, duration=None, max_msgs=-1):
        self.msgs = queue.Queue(maxsize=max_msgs)

        self.subscibers = []
        for topic, msg_type in zip(self._topics, self._msg_types):
            sub = message_filters.Subscriber(topic, msg_type)
            self.subscibers.append(sub)

        self.__ts = message_filters.ApproximateTimeSynchronizer(
            self.subscibers,
            self._queue_size,
            self._sync_tolerance,
            allow_headerless=True,
        )
        self.__ts.registerCallback(self._msg_callback)

        if duration:
            rospy.sleep(duration)
            return self.stop()

    def stop(self):
        [sub.sub.unregister() for sub in self.subscibers]
        self.__ts = None
        return list(self.msgs.queue)

    def _msg_callback(self, *args):
        self.msgs.put(args)
