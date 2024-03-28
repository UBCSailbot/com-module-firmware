# Git Project Workflow Instructions

The following outlines how we can use GitHub to streamline the firmware development process. We first have step-by-step instructions on how to use GitHub issues to your advantage, and then we have Git commands, which are to be used in either GitBash or Terminal. Note that for some of the commands listed, 

## Workflow using Issues
ChatGPT says: "GitHub issues are a feature of the GitHub platform that allows users to track tasks, enhancements, bugs, or any other kind of item related to a repository. They serve as a centralized communication and coordination tool for managing project tasks and discussions among team members or contributors."

Here are the steps you and your team can take to use issues effectively:
1. Create an issue which describes the task that needs to be done
2. Create a new branch attached to the issue
3. Complete the task in the new branch
4. Open a pull request and add a reviewer (usually the person who made the issue, or your lead)
5. Reviewer makes change requests (if they have any)
6. Address the request in the same branch
7. Request another review and repeat steps 5-7 as necessary
8. Once approved by the reviewer, the pull request can be merged with the main branch (see the section below "Completing a Pull Request"). The issue linked to the pull request will close, and the branch will be automatically deleted. Good work!

## Cloning an Online Repository
1. ```git clone repo_url```
2. ```cd repo_name``` (cd stands for change directory - this takes you into the repo)

Note that for us, repo_url is https://github.com/UBCSailbot/com-module-firmware.git and the repo_name is com-module-firmware. You always have to run these first two commands the first time you open GitBash or Terminal.

## Pulling from Repository
```git fetch``` (Updates your local repository with any changes made in the remote repository)

```git pull``` (Performs what ```git fetch``` does, and then merges those changes with your current branch)

## Pushing to Repository
1. ```git add -A``` (The -A adds all the changed files in the project)  
OR  
```git add file_name```
2. ```git commit -m message``` (Replace "message" with a description of the changes made. Make sure to commit often to avoid unintentionally losing changes!)  
3. ```git push``` (If you are pushing a new branch for the first time use the command listed under the Dealing with Branches section)

## Dealing with Branches
- ```git branch``` (Lists all local and remote branches)
- ```git branch branch_name``` (Creates a new branch)
- ```git checkout branch_name``` (Switches you to a specified branch)
- ```git checkout -B branch_name``` (Creates a new branch AND switches you to it)
- ```git push -u origin branch_name``` (The new branch will first be on your local machine, and will show up on the GitHub repo once you run this command. In subsequent pushes you can simply use ```git push``` as listed under the Pushing to Repository section)

### Completing a Pull Request
1. ```git merge main``` (This synchronizes your branch with any changes that may have been made to main in the time since you created your branch. If there are conflicts in any files, then it will let you choose to either keep the file in your branch or the one in main)
2. To finish, merging the pull request with main is done on the GitHub website. Simply navigate to the pull request and follow the steps to merge it.

## Other Useful Commands
- ```git status``` (Lets you see which changes have been staged, which haven't, and which files aren't being tracked by Git)

