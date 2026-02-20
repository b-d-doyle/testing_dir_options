# testing_dir_options
I will definitely delete this. I'm testing options before I pick one for UCF_CMR

# bash commands:
Add a repo to this repo:
```
git subtree add --prefix=<local-directory> <remote-url> <remote-branch> --squash
```
Get updates from a subtree'd repo (No permissions required):
```
git subtree pull --prefix=<local-directory> <remote-url> <remote-branch>
```
Push updates to a subtree'd repo (Only for your own repo):
```
git subtree push --prefix=<local-directory> <remote-url> <remote-branch>
```
