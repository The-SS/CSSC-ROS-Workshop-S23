# CSSC-ROS-Workshop-S23
Spring 2023 ROS workshop for CSSC at UTD.
- Date: Friday March 10, 2023
- Time: 1:00PM-2:30PM CST
- Location: ECSW 3.210

Agenda: 
- 30min presentation introducing ROS, its basics, and how it is used
- 10min ROS demo
- 50min Q/A + hands-on tutorial 

To participate in the tutorial, please bring a laptop configured as described below.

## Set up your laptop with Ubuntu 20.04
For the tutorial, you need a computer running Linux with ROS installed. If you have a Windows or Mac machine, we recommend setting up a virtual machine with Linux as discussed below. Then, ROS can be installed (see below).

You can use [this tutorial](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview) to set up Ubuntu with VirtualBox 7 or follow the steps below.

### Setting up a Virtual Machine
We recommend using VirtualBox to setup a the virtual machine and using Ubuntu 20.04. 
1. [Download VirtualBox](https://www.virtualbox.org/).
2. [Download Ubuntu 20.04 "Dektop image"](https://releases.ubuntu.com/focal/).
3. Launch VirtualBox.
4. Click "New" to create a new virtual machine.
5. Give it a name (e.g. Ubuntu 20) and make sure the Type is set to "Linux" and the Version to "Ubuntu (64-bit). Click next.
6. Select the amount of RAM to give the virtual machine (4096MB should be enough). Click next.
7. Select "Create a virtual hard disk now", click create, then select "VDI (VirtualBox Disk Image)" and hit Next.
8. Choose "Dynamically allocated" for storage on physical hard disk. Click next.
9. Choose how much disk space to allocate. 15GB should be enough. Click create.
10. Double click your newly created vitrual machine and wait for a new window to pop up.
11. Select the Ubuntu 20.04 '.iso' image that you downloaded for the start-up disk and click Start.

The virtual machine should now start. Move on to installing Ubuntu.

### Installing Ubuntu 20.04
1. Select "Install Ubuntu" from the install options.
2. Select your keyboard layout (probably "English (US)"). Click continue.
3. Choose "Normal installation" and "Download updates while installing Ubuntu. Click continue.
4. Select "Erase disk and install Ubuntu". Click "Install Now".
5. Click continue in the pop-up window ("write the changes to disks?").
6. Select your timezone ("Chicago"). Click continue.
7. Choose your name, computer name, username, and password ('demo' for all should be fine). You can also choose "Log in automatically". Click continue.
8. Wait for the installation to finish.
9. Click "restart now" when the installation is done and just hit "ENTER" when prompted to do so.

## Installing ROS
Since we used Ubuntu 20.04, we will install ROS Noetic.
Follow [these instructions](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS Noetic and go with the "Desktop-Full Install: (Recommended)" option. When doing step "1.5 Environment setup" follow instructions for Bash not zsh. 

If you get an error related to your user not being part of the sudo users group, run the following commands in terminal replacing [username] by your user name (e.g. demo).
```
su -
sudo adduser [username] sudo
```



















