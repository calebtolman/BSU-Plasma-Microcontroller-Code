# BSU-Plasma-Microcontroller-Code

Here are instructions how to commit to the repository.
Github is a really good way to share code, rather than something like Google Drive.
I am not an expert on git, but I think this will help.


## Commiting and Pushing

To commit to this repository, put the file you want in a folder on your computer.

Open a terminal, make sure you have git installed by entering 'git --version' without quotes.\
  if your git version is returned, then you have git. Otherwise install git.

Navigate to the folder with the file you want to commit. Do this by using 'cd [directory name]'\
  Ex. if the folder is in Users/YourName/Desktop/MyFolder,\
  and you are in Users/YourName, then type\
  cd Desktop\
  cd MyFolder\
  or cd Desktop/MyFolder\
      you can go back a directory by using cd ..\
      Ex. Users/YourName/Desktop/MyFolder/ThisFolder\
      cd ..\
      => Users/YourName/Desktop/MyFolder

  You can return the contents of the folder with 'ls'
  
Once you are in the right place, type 'git clone [repository URL]'. Get the repository URL from the github repository by clicking button labeled '<> Clone'
  This will configure your local folder with the github repo. It also makes a new folder in the directory you are in with all the stuff from the github repo, Ex. MyFolder/BSU-Plasma-Microcontroller-Code. Move the files in the new folder to the first folder. Delete the empty new folder.

Now, to commit to the repo with your new file, make sure you are in the correct directory on your computer, Ex. MyFolder/, and it has all the files that are currently in the github repo + the files you want to put in the github repo.\
  Type 'git add .' The dot is so it stages all of the files in the directory you are in.\
  Type 'git commit -m "I am committing!"' This will set your commit message - "I am committing!"\
  Type 'git push origin main' This will push your commit to the github repo and your files should be on github after this.

If your local repository is not up to date with the one on github, then you might have to do 'git pull' before you can commit and push with the steps above.

If we are all working on the same file at the same time, then we may run into some merge conflicts, but we will figure it out. 


## Using .gitignore

If you have files on your local repo that you do not want to commit to the github repo, then make a text file called '.gitignore' and put it in the same directory as your local repository. Put the names of the files you want to be ignored seperated by a new line.\
  Ex.\
  ThisFolder/ <--- this is a folder\
  .project\
  MyFile.txt\
  *.class <--- this ignores all files with a .class extension\
  .gitignore <--- ignores .gitignore

.gitignore helps so if you are working on the file in your IDE, you can just commit the file in the workspace folder of your IDE and not have to copy/paste the file you are working on into another folder to commit it, you can just edit the file in your IDE and commit it from the folder it is in.


## Other helpful commands

'git remote -v', checks the github repo your local repo is hooked up to\
'git fetch', shows new commits made to the github repo, but does not pull them like 'git pull'\
'git status', shows your untracked files, the ones that will be commited.\
'pwd', print working directory. Ex. Desktop/MyFolder\
'mkdir [directory name]', make a new directory (folder)
