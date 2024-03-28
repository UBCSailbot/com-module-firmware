# Git Project Workflow Instructions

The following are commands to be used in either GitBash or Terminal. Using Git in this fashion allows everyone to keep their work organized and avoid overlap.

## Cloning an Online Repository
1. ```git clone repo_url```
2. ```cd repo_name``` (cd stands for change directory - this takes you into the repo)

Note that for us, repo_url is https://github.com/UBCSailbot/com-module-firmware.git
and the repo_name is com-module-firmware.

## Pulling from Repository
```git pull```

## Pushing to Repository
1. ```git add -A``` (The -A adds all the changed files in the project)  
OR  
```git add file_name```
2. ```git commit -m message``` (Replace "message" with a description of the changes made)  
3. ```git push``` (If you are pushing a new branch for the first time use the command listed under the Dealing with Branches section)

## Dealing with Branches
- ```git branch``` (Lists all local and remote branches)
- ```git branch branch_name``` (Creates a new branch)
- ```git checkout branch_name``` (Switches you to a specified branch)
- ```git checkout -B branch_name``` (Creates a new branch AND switches you to it)
- ```git push -u origin branch_name``` (The new branch will first be on your local machine, and will show up on the GitHub repo once you run this command. In subsequent pushes you can simply use ```git push``` as listed under the Pushing to Repository section)



## Other Useful Commands
- ```git status``` (Lets you see which changes have been staged, which haven't, and which files aren't being tracked by Git)  

