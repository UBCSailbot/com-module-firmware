# Git Project Workflow Instructions

The following are commands to be used in either GitBash or Terminal. Using Git in this fashion allows everyone to keep their work organized and avoid overlap.

## Cloning an Online Repository
```git clone repo_url``` (Replace "repo_url" with the repository url)

## Pulling from Repository
```git pull```

## Pushing to Repository
1. ```git add -A``` (The -A adds all the changed files in the project)  
OR  
```git add file_name``` (Replace "file_name" with the file you want to commit)
2. ```git commit -m message``` (Replace "message" with a description of the changes made)  
3. ```git push```

## Creating a New Branch 
```git checkout -B branch_name``` (Replace "branch_name" to the name of your choice)

## Other Useful Commands
- ```git status``` (Lets you see which changes have been staged, which haven't, and which files aren't being tracked by Git)  
- ```git checkout branch_name``` (Lets you navigate between the branches. Replace "branch_name" with the branch you want to switch to, e.g. main)  
- ```git branch``` (Lists all local and remote branches)
