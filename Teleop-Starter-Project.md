# Introduction

This project will involve the creation of a drive control test bench, in which you will create an override for the joystick that we use to control the rover, send these joystick values to a backend where they will be mathematically converted to drivetrain values, then display these drivetrain values back on the GUI.

In this tutorial any code that is surrounded by \*asteriks\* is code that you are meant to add to the project in a certain spot. Please make sure you understand what you are adding when you add it and ask questions as you go. This project is solely for your learning so take as much time as you need to understand what is actually happening as you go through it.

# Getting Started

First, checkout the branch that has the starter project code:

```bash
git checkout teleop-starter
```

Then checkout a new branch to work on:

```bash
git checkout -b <your initials>/starter-project
example: git checkout -b tjn/starter-project
```

This will create a new branch so that you will not be changing your master branch in your git repository. You will NOT be pushing this branch, it is just there to keep it separate from your master branch.
Whenever you checkout a new branch, make modifications to ROS messages, or modify the CMakeList files you need to build the Mrover ROS2 worksapce. To do this, run `./build.sh` in the `mrover` directory. In order to ensure Vue has all the dependencies/libraries it needs installed, go to the `starter_project/teleop/basestation_gui/frontend` folder and run the command `bun install`. If you do not have bun installed, run `curl -fsSL https://bun.sh/install | bash` first. Bun is a package manager and runs Javascript for us, which allows us to run the frontend.
To start the project, you will launch it with a launch file much like the main base station GUI. Ensure you're in the `starter_project/teleop` folder:

```bash
/home/<USER>/ros2_ws/src/mrover/starter_project/teleop $ ros2 launch mrover teleop_starter_launch.py
```

Then navigate to `localhost:8080` in your web browser.

# The Starter Code

Now that you have cloned the starter code and have set up your workstation, you are ready to begin the actual project. There are a few things in the starter code to take a look at.
<br/>
First let's take a look at launch/teleop_starter_launch.py:
<br/>

```python
    backend_node = Node(
            package="mrover",
            executable="gui_starter_backend.sh",
            name="teleop_backend"
    )

    frontend_node = Node(
            package="mrover",
            executable="gui_starter_frontend.sh",
            name="teleop_frontend"
    )
```

This runs gui_starter_backend.sh, a bash script which starts the Django app (our GUI). The other bash script starts the Vue.js frontend. Luckily, both applications automatically refresh after you make changes, so you shouldn't have to stop running this launch command when editing code.
<br/><br/>
Next let's take a look at package.json:

```json
  "devDependencies": {
    "@babel/core": "^7.12.16",
    "@babel/eslint-parser": "^7.12.16",
    "@vue/cli-plugin-babel": "~5.0.0",
    "@vue/cli-plugin-eslint": "~5.0.0",
    ...
```

This file defines the packages that bun installs when you run `bun install`. You shouldn't need to touch this at all. However, if you need to ever use a dependency (such as a styling like bootstrap), run `bun install` in the frontend directory and it should look through the repo and install/update dependencies.
<br/><br/>
Next let's look at the frontend/src/components directory as well as router/index.js. MenuPage.vue defines the main menu that can be found at the default route of our program. <br/><br/> DriveControls.vue has some basic code to get values from the Joystick that is normally used to operate the rover.

```javascript
interval = window.setInterval(() => {
	const gamepads = navigator.getGamepads();
	const gamepad = gamepads.find(
		(gamepad) => gamepad && gamepad.id.includes("Thrustmaster")
	);
	if (!gamepad) return;
}, updateRate * 1000);
```

It's encouraged that you explore other files but these are the essential spots that are important to the starter code.
Let's begin working on the motor sim!

# Adding a route for the drive controls

First we are going to add a separate route, /motor_sim for the drive controls vue module. A route is simply a separate page under the same website. For example, for project management we use https://github.com, but our repo is located at https://github.com/umrover/mrover-ros2, "umrover/mrover-ros2" is the route to our repo.

To add our route we are going to add a MenuButton to the MenuPage component.

```html
<div>
	<MenuButton link="/motor_sim" name="Motor Simulator"></MenuButton>
</div>
```

This will create a menu button that will link to the motor_sim route. The template section of the code should now look like:

```html
<template>
	<div class="wrapper">
		<div class="shadow p-3 mb-5 header">
			<h1>Menu</h1>
			<img
				class="logo"
				src="/mrover.png"
				alt="MRover"
				title="MRover"
				width="200"
			/>
		</div>
		<div>
			<MenuButton link="/motor_sim" name="Motor Simulator"></MenuButton>
		</div>
	</div>
</template>
```

Now we need to add the MenuButton to the components of the vue module. Add the following line to the top of the script section of the code

```javascript
import MenuButton from './MenuButton.vue';

export default {
...
```

then add a components section to the export section

```javascript
export default {
	name: "MenuPage",

	components: {
		MenuButton,
	},
};
```

Your menu will now look like this:
<br/>
![image](https://user-images.githubusercontent.com/71604997/188680144-724e2145-3ba5-48e5-b25e-42cc5fed8122.png)
<br/>
You'll likely notice that when you click the button it takes you to a blank page. This is because we need to define the route. Open the src/router/index.js file. We are going to add a route that looks very much like the one that is already present for the menu.

```javascript
import { createWebHistory, createRouter } from "vue-router";
import Menu from "../components/MenuPage.vue";

import DriveControls from "../components/DriveControls.vue";

const routes = [
	{
		path: "/",
		name: "Menu",
		component: Menu,
	},
	{
		path: "/motor_sim",
		name: "Motor Sim",
		component: DriveControls,
	},
];

const router = createRouter({
	history: createWebHistory(),
	routes,
});

export default router;
```

"Drive Controls" should now be displayed on the /motor_sim route.

# Sending a message to Django

Head over to DriveControls.vue. Now you need to send joystick values from Vue to Django:

```javascript
created: function() {
    this.interval = window.setInterval(() => {
      /*
        To test this code either plug in the Thrustmaster and use this code:
      */

      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Thrustmaster'))
      if (!gamepad) return

      this.sendMessage({
        type: 'joystick',
        axes: gamepad.axes,
        buttons: gamepad.buttons.map(button => button.value)
      })


      /*
        OR, test the code with hardcoded values:
      */
      const axes = [0, 0.75, 0.1, 0.5, 0, 0] // index 1 affects forward/back, 2 affects twist/turn, 3 affects throttle
      const buttons = Array(16).fill(0) // 16 buttons available to map to. None of them are "pressed" currently

      this.sendMessage({
        type: 'joystick',
        axes: axes,
        buttons: buttons
      })

    }, 1000 / UPDATE_HZ)
  }
...
```

`this.sendMessage()` sends a JSON message to the Websocket stored in the Vuex store. This websocket will send the message to the Django backend.

# Processing our joystick values

Now that the joystick values are sent to Django, they need to be mapped to the correct buttons and axes the operators expect as well as filter the input. Head over to backend/consumers.py. This code has been provided for you since it is new and complex. Last year, we had issues with axes swapping on the gamepads in Javascript, so a member created a class to handle input. As you can see below, the `receive()` function, which receives the JSON messages from the Websocket will send the joystick inputs to this function. This function will ultimately publish a `Twist()` message to ROS on the topic `/joystick_cmd_vel`. Feel free to look at joystick handling function more in depth.

```python
    def receive(self, text_data):
        ...
        match message:
                case {
                    "type": "joystick",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_joystick_twist(device_input)
                case _:
                    rospy.logwarn(f"Unhandled message: {message}")
        ...

```

# Adding a custom ROS Topic Message

Since the joystick values are being published to a ROS topic, we can now simulate what ESW does: calculate and publish motor output commands. To do this, first we are going to create a custom ROS message file. ROS has many msg types defined such as `Twist` which is what the joystick publisher publishes, but many times we want to send a custom messages on topics. To begin, create a new file called WheelCmd.msg inside the `starter_project/teleop/msg` folder.
<br/>
Your workspace should now look like this:
<br/>
<br/>
![image](https://github.com/user-attachments/assets/9c04ca8c-0539-45be-856d-916daa95d173)
<br/>
<br/>
Inside the WheelCmd.msg file we will define the members of the message we are going to be sending:

```
float64 left
float64 right
```

Note that these message files use types according to the ROS message format defined [here](http://wiki.ros.org/msg#Fields).
<br/>

Now navigate back to the mrover directory and run `./build.sh` again to add your message to the mrover package. You must run `./build.sh` any time a msg or srv is created or modified!

# Transform joystick values to motor outputs

Currently, our interface can send Vue data and publish to ROS, but what about the other way? Next, we'll create a subscriber to listen to the /wheel_cmd topic and then send that data to Vue through the websocket. Add this code to backend/consumers.py.

```python
class GUIConsumer(JsonWebsocketConsumer):
    def connect(self) -> None:
        self.accept()

        ########################################################################################
        # Use self.forward_ros_topic when you want to get data from a ROS topic to a GUI
        # without needing any modifications done on it. For instance, reading motor output.
        ########################################################################################
        self.forward_ros_topic("/wheel_cmd", WheelCmd, "wheel_cmd")
```

This function was created by one of our members and makes it easy to send an entire ROS message through the websocket to Vue.

Now Import your messages at the top of the python script

```python
from mrover.msg import WheelCmd
```

# Adding Motor Outputs to the GUI

The final step will be to display the output values that are given for the motors in order to observe the outputs of whatever algorithm you chose.

Head back to DriveControls.vue to begin on this.

The first step to doing this will be adding variables to hold the most recent wheel_cmd values.

```javascript
export default {
  data() {
    return {
      left: 0,
      right: 0,
    }
  },
```

There are a few things to note about this code block. For one, all values declared in the data() section need to be prefixed with "this." when accessing them at any time. This is because they are member variables of the Vue component and thus can be accessed in any context and are not just local to a certain function or context (It's a very common javascript syntax).
Now we will display these values on the GUI.

```html
<div>
	<p>Drive Controls</p>
	<div>
		<p>Left Motor Output: {{left}}</p>
		<p>Right Motor Output: {{right}}</p>
	</div>
</div>
```

Putting things in double curly braces will output the member value of the Vue component with the same name. Note that since this only works for member components, you do not need to use "this."

Since we sent the wheel_cmd values from Django to Vue, we need to assign the left and right values as each message is received. This can be done through the watcher of the message variable stored in the Vuex store. This message variable stores the current message received in the websocket.

```javascript
data() {
    return {
      left: 0,
      right: 0,
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'wheel_cmd') {
        left = msg.left;
        right = msg.right;
      }
    }
  },
...
```

# Testing your Motor Simulator

You may have been asking yourself through this starter project "How am I going to test this code?" Well, either use a joystick or hard-coded joystick values to test in DriveControls.vue. In order to test the code, run the teleop_starter_launch.py to launch the frontend and backend:

```bash
ros2 launch mrover teleop_starter_launch.py
```

In another terminal, run the `motor_sim.py` script. This creates another ROS node which subscribes to the joystick commands topic and simulates what ESW does (calculates a left and right motor output and publishes to /wheel_cmd).

```bash
ros2 run mrover motor_sim.py
```

OR, you can go further and use the beauty of ROS. You can create a custom Joystick message with axes and buttons and put a subscriber in consumers.py with the callback being the function `send_joystick_twist`. Them, you can use rostopic to publish custom messages. The rostopic command line utility allows us to send our own custom messages to any ros topic using only the terminal.

You can send a Joystick messages by entering:

```bash
ros2 pub /joystick mrover/Joystick '{axes : [0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}'
```

To test if you're receiving WheelCmds, you can also use ros2 topic, but publish a simple Twist message:

```bash
ros2 topic pub /joystick_cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]''
```

ros2 topic pub is a super helpful tool for debugging, the other most important command to know about is ros2 topic echo. After sending your joystick message run the following command in another terminal:

```bash
ros2 topic echo /wheel_cmd
```

This should display the same values that are displayed on the GUI end of things. You can learn more about ros2 topic [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).

With this, you have finished the teleop starter project and hopefully should feel comfortable writing code that can be used to control the rover. Congratulations!
