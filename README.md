# Summary
This project is targeted for CS4750: Foundations of Robotics at Cornell University (it's a really cool course, go take it). 

I'm trying to address the eternal VM issues that plague both the students and TAs by creating a containerized ROS environment preloaded with the dependencies for the class.
It's also a great way to use continue using ROS after you've completed the class! 

The container runs ROS Noetic and uses either [noVNC](https://novnc.com/info.html) or [X11 forwarding](https://goteleport.com/blog/x11-forwarding/) for GUI apps (namely RVIZ).

# Compatability

- [x] Linux (tested on Ubuntu 22.04.3 LTS x86_64)
- [x] Windows (untested but should work. Let me know if you give it a shot)
- [x] Mac (untested but should work. Let me know if you give it a shot)

# Quick install
1. Install docker for your system:
https://docs.docker.com/get-docker/

2. Open (or install) VSCode

3. In VSCode, install the "Dev Containers" extension from Microsoft.

4. In VSCode, View -> Command Palette -> (type) Dev Containers: Rebuild and Reopen in Container

5. Wait patiently, Docker is building your development environment. This step will take several minutes the first time, but will take less than a second on subsequent connections.

6. Visit http://localhost:8080/vnc.html to see the system GUI. It's recommended you only use this GUI to interact with GUI applications (RVis) and use the VSCode terminal (Terminal -> new terminal) to run your commands.
It's much faster.

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

# X11 forwarding
