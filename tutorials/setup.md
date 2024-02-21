# STM32 Environment Setup
This tutorial is intended to setup an inital Sailbot STM32 development environment along with git version control. We assume you are using a Windows operating system. Before we begin, make sure your system has the following prerequisite tools.
## Prerequisites
List of tools required to get started:
- STM32Cube IDE - Used to generate initial code and peripherial configurations using the in-built STM32CubeMX.
- Visual Studio Code - Alternative IDE for editting code option to Cube IDE. (optional)
- Putty - Used for serial communication between host and STM32U5 board.
- Git - Version control.

## Initial Setup
Once you have all the tools ready, we can begin initializing you coding environment.
1. Open STM32CubeIDE
2. You will be promted to select your workspace. A workspace is a directory that will contain your STM32 projects. If you have not created it before click 'launch' and it should automatically create a workspace in ``C:\Users\Sailbot\STM32CubeIDE\workspace_1.14.1``
3. Open the directory of your workspace (default: ```C:\Users\Sailbot\STM32CubeIDE\workspace_1.14.1```) on your file explorer
4. Right click then 'Git Bash Here' to open up your git bash terminal

![image](https://github.com/UBCSailbot/com-module-firmware/assets/71032077/d07fd6f6-2b90-4bb4-9f1c-92c2c5dd0344)
5. Clone this repo by typing in the command ```git clone https://github.com/UBCSailbot/com-module-firmware.git```
6. You will be in the main branch (Optional you an switch branches using ```git checkout branchName```)
7. Open up CubeIDE again and leave it open on the side
8. Go to a project folder that you want to work on e.g project/base-library/project/
9. Double click the ```.project``` file
10. The Cube IDE should prompt a success and the project should be added to your workspace (under the project explorer)

![image](https://github.com/UBCSailbot/com-module-firmware/assets/71032077/9cea3738-9b28-42d3-a595-6867d5e13b06)

You have now learnt how to add a project to your workspace. You can create multiple workspaces in STM32 CubeIDE. A workplace can contain projects from any location on your local device and the mapping is stored inside the ```.metadata``` directory. Usually I like to have a workspace that is dedicated to only contain test projects created under the 'tests' directory.

## Create a new project
To create a new project under ```projects/``` 
1. Duplicate the preexisting project template directory ```projects/template``` and change the name to whatever your project is called
2. Template folder contains all the default configuration and generated files for the STM32U5
3. Navigate to the ```project/``` directory under your newly created project
4. Open up your workspace in CubeIDE
5. Click on ```.project``` file to add this project to your workspace
6. The name of the project as seen in the project explorer tab should be "template"
7. Right click the project on the project explorer tab and click rename
8. Set the name to whatever your project is called and ensure that "Update references" is checked
9. Click "Ok"
10. To verify that everything is good run a build. Right click the project and bulid (Or Ctr-B)
11. You can then open up the ".ioc" file to modify your Pinout and configurations as per project requirements

## Create a new test
Inside your projects tests directory, there should contain a test stm32 project template. Follow similar steps listed in the above section to create a new test project.

## Start writing code
STM32CubeMX generated ```Core``` directory is where user source code will be written for you applicaion. The ```Drivers``` location contains all the STM32U5 Hardware Abstraction Layer (HAL) drivers, you most probably won't ever need to modify them. 
TODO: Walk through the development process. Would be ideal to create seperate folder under ```src/``` that contains all the user codebase instead of dumping them under ```src/```.


## Project Workflow
TODO: Possibily create another markdown file detailing the github workflow



 
