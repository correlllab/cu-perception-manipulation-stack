from teleop_interface.msg import object as ob

class IdentifiedObjects(object):
	def __init__(self):
        self.nh = rospy.init_node('placing')
        self.ob_sub = rospy.Subscriber('/identified_objects', ob,
                                       self.received_object, queue_size=1)
        self.objects = dict()

    def received_object(self, msg):
        self.objects[msg.object_identity] = msg

