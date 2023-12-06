# Summary
This project is targeted for CS4750: Foundations of Robotics at Cornell University (it's a really cool course, go take it). 

Over the course of the class, the VMs were a constant annoyance. By creating a containerized CS4750 ROS environment, I hope to allow students to run ROS locally for a more enjoyable experience.
It's also a great way to use continue using ROS after you've completed the class! 

The container runs ROS Noetic and uses either [noVNC](https://novnc.com/info.html) or [X11 forwarding](https://goteleport.com/blog/x11-forwarding/) for GUI apps (namely RVIZ).

# Compatability

- [x] Linux (tested on Ubuntu 22.04.3 LTS x86_64, and used to complete all of the CS4750 projects)
- [x] Windows (untested but should work. Let me know if you give it a shot)
- [x] Mac (untested but should work. Let me know if you give it a shot)

# Quick install
1. Clone this repository (dowload and unzip, or `git clone https://github.com/usedhondacivic/ROS-docker-container.git`)
  
2. Install docker for your system:
https://docs.docker.com/get-docker/

3. Open (or install and open) VSCode

4. In VSCode, install the "Dev Containers" extension from Microsoft.

5. In VSCode, View -> Command Palette -> (type) Dev Containers: Rebuild and Reopen in Container

6. Wait patiently, Docker is building your development environment. This step will take several minutes the first time, but will take less than a second on subsequent connections.

7. Open a VSCode terminal (Terminal -> new terminal). It should say `lcl up` on the left hand side. Run `rosrun rviz rviz`.

8. Visit http://localhost:8080/vnc.html to see the system GUI. It's recommended you only use this GUI to interact with GUI applications (RVis) and use the VSCode terminal to run your commands.
It's much faster.

9. You should see RViz open in your browser! Now you can copy your own homework files into homework_ws/src, and they will be immediately available within the container!

Have a need for speed? noVNC is definitely slower than native apps (but still much better than the VMs because it's on your local network).
You can instead use X11 forwarding to render the GUI apps natively, but it requires more configuration. See the X11 forwarding section below for details on the setup.

# About the project

## Why?
As you may have noticed, VM's are frustrating. They are painfully slow, require you to connect to the Cornell network over VPN, and obfuscate the configuration of the project. 
On top of all that, once the semester ends you will have no way to run the code you worked so hard on!

What we really want is a setup that is:
  * Portable (all dependencies and code are packaged together)
  * Cross platform (cause installing a specific version of Ubuntu for each ROS release is ridiculous)
  * Easily configurable (theres so much more to be done with ROS!)

These three goals have been troubling computer scientists for decades, and luckily for us they've come up with a nifty solution: containers.

## How?
Containers create an isolated environment for building and running software.
[They're conceptialy very similar to VM's](https://www.atlassian.com/microservices/cloud-computing/containers-vs-vms), with the main difference being that VM's virtualize an entire computers hardware while containers only virtualize the operating system and the software levels above that. As a reult they are more lightweight and efficent than VM's. They are also a lot easier to setup.

Docker is a popular software for creating, maintaining, and distributing containers, and is the one I used for this project.

Checkout the `DockerFile` in the root of the project. In it are all the commands that are used to build the container. Every single dependency is stored in that file, and will stay the same unless you change them. The first line pulls the official Noetic ROS image, which contains the necessary version of Ubuntu and ROS for the project.

The `DockerFile` is built using Docker Compose, a tool for using multiple containers at the same time. ROS is run in one container, noVNC in another, and the communication pathways are setup in `DockerCompose.yml`.

Finally, VSCode integration is accomplished using the Dev Containers workflow. The file `.devcontainer/devcontainer.json` tells VSCode how to run the containers. When we request VSCode to attach to the container, it build the container for you and injects a VSCode server into the system. That server communicates back to your local VSCode install, allowing it to act as if you were running VSCode natively within the container environment.

# X11 forwarding

Still working on documenting this, come back soon.

# Known issues
In the arm final project (F23 semester), I was unable to fix the following error:
```
You can start planning now!

Unknown tag "material" in /robot[@name='robot']/link[@name='floor']/collision[1]
[INFO] [1701661907.915211]: The operating mode for the 'all' group was changed to position.
[INFO] [1701661907.915965]: The operating mode for the 'arm' group was changed to position.
[ERROR] [1701661925.869469500]: Pose frame 'wx250s/ee_gripper_link' does not exist.
[ERROR] [1701661926.041593854]: Different number of names and positions in JointState message: 6, 0
```

# Debugging

Break something in your config? No worries! Simply save you homework folders somewhere. Delete the ROS-docker-container folder, and follow the installation instructions again.

# Conclusion
Thanks for reading, check out my other projects! https://michael-crum.com/
