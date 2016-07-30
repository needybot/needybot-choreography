# Needybot Choreography 
Actionlib package for animating Needy's mobile base

## Quick Start
Download the package to your catkin workspace and run `catkin_make`

    $ cd ~/catkin_ws
    $ git clone https://github.com/needybot/needybot-choreography.git
    $ catkin_make

Add the animator node to your launch file

    <node pkg="needybot_choreography" type="animator.py" name="animator" output="screen"/>

In your code, give Needybot an animation goal

    import actionlib
    from needybot_msgs.msg import AnimatorGoal

    self.goal = AnimatorGoal(animation=AnimatorGoal.SPIN)
    self.client = actionlib.SimpleActionClient('animator', AnimatorAction)
    self.client.wait_for_server()
    self.client.send_goal(self.goal)
    self.client.wait_for_result()
    # Do something when animation is completed

Thats it!
