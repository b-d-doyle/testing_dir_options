# testing_dir_options
I will definitely delete this. I'm testing options before I pick one for UCF_CMR

# bash commands:
Add a repo to this repo:
```
git subtree add --prefix=<local-directory> <remote-url> <remote-branch> --squash
```
Get updates from a subtree'd repo (No permissions required): --squash
```
git subtree pull --prefix=<local-directory> <remote-url> <remote-branch>
```
Push updates to a subtree'd repo (Permissions required; only for your own remote repos):
```
git subtree push --prefix=<local-directory> <remote-url> <remote-branch>
```
You can also pull from this repo to your remote repo (I hope. Instructions coming)

# Included Repos:
Put links to the repos here -- you'll need them for the above commands

Lesker275_logger:
> https://github.com/b-d-doyle/Lesker275_logger.git

MKS925_serialRead:
> https://github.com/b-d-doyle/MKS925_serialRead.git

RF_Power_Control:
> https://github.com/b-d-doyle/RF_Power_Control.git
