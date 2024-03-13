# Robotics Lab @UNISA

Welcome to the Robotics lab of the Automatic Control Group!

This repository collects the official software of the lab, needed to operate the robots and support teaching activities.
The repository is thought for students with the aim to provide a representantive environment of a typical setup that can be found in enterprises.
The same software is also used for the development of master theses projects and some research activities.

## Manifesto

Robotics is a broad interdisciplinary field, where many branches of knowledge merge to build complex systems.
For this reason, no single person or institution can develop code to cover all the possible needs that arise in robotics.
By definition, robotic software development is **collaborative**.

The *Robot Operating System (ROS)* is an example of an open-source collaborative framework for robotics.
While using ROS2 as a backend, this repository is another (smaller) example of a collaborative software development environment, and specifically targets the robots and applications that are of interest of the Automatic Control Group @UNISA.

**This repository is written by (mostly) students, for students**.
It is the result of the work of each single contributor who, seriously and meticulously, committed quality software components, with a vision that went beyond his/her exam or thesis project.
If more complex and interesting applications can be built at the Robotics Lab and more interesting lessons and examples can be delivered to students, it is thanks to their work.
If, during your exam or thesis project, you took advantage of the code written by someone else, that helped in achieving your objective and passing your exam, it happened because someone before you worked well.
**You have the moral duty of doing the same!**

If you feel that you need to write low quality software to gain speed, **please read [this article](https://anthonysciamanna.com/2016/11/27/the-code-quality-versus-speed-fallacy.html)**.
In short: low quality software makes technical debt accumulate.
Paying back your debts always costs more than taking no debt!

## Contributing to the development

Here are some guidelines to contribute to the development of this repository.

### Branching model

The branching model used in this repository is [GitFlow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).

#### Branch types

This model foresees a set of well known branch names and types.
These are:

* **master**: This branch is an ever lasting branch that includes all released software versions
* **develop**: This branch is an ever lasting branch that includes all working and deployable software
* **feature**: This is a branch category (feature/<branch_name>) that is used when the developer is adding a new functionality.
This includes refactorings that are aimed to make software more maintainable and extendable.
* **bugfix**: This is a branch category (bugfix/<branch_name>) that is aimed to solve a bug and branches out from develop.
* **hotfix**: This is a branch category (hotfix/<branch_name>) that is aimed to solve a bug that is found on released software (on the master branch).
* **release**: This is a branch category (release/<branch_name>) that is aimed to freeze the baseline for a software release that will be merged to the master branch.
This branch family branches out from the develop.

**Branch scope**: branch scopes should be limited and focused as much as possible to accomplish a specific goal.

**Branch names**: branches should be named after their goal.
Their name should not exceed 4 words and should be in [snake case](https://it.wikipedia.org/wiki/Snake_case).

#### Branching dynamics

As part of the gitflow workflow there is a scheme that shall be followed when forking and merging the branch types mentioned before:

* **master**: starts from the root of the tree, it is the main development branch therefore it is not merged into other branches
* **develop**: branches off from master very early in the git log, it merges back into master indirectly through a release branch
* **feature**: are created from develop or from other feature branches and merge back on develop.
It is suggested to branch from feature branches only temporarily until the feature branch is merged into develop.
* **bugfix**: follows the same branching pattern of feature branches
* **hotfix**: branches off from master and merges back to master.
* **release**: branches off from develop and merges both into master and develop

#### Development and review workflow

1. Identify your bugfix or feature and **agree a branch name** `<name>` with a repository administrator.
2. Identify the branch to start from, either `develop` or another feature or bugfix branch.
3. Develop your feature or bugfix and their tests, **write its documentation** (typically a user manual, important maintenance information and known issues, if any)
4. **Execute tests** (unless they are automatically executed) and review the documentation
5. Upon successful tests, request the merge of the branch onto the `develop` through opening a **Pull Request (PR)**.
PRs officially start the review process.
6. Reviewers generate comments requiring modifications of the source code, test code, configuration and/or documentation.
7. Discuss with reviewers through the PR and implement their comments.
8. Upon implementation of comments, reviewers approve the PR or request further changes.
9. Upon reviewers approvals, administrators merge the feature or bugfix branch onto the `develop`.

### Commit messages

Configure git for using the .gitmessage as commit template issuing the following command:

```bash
git config commit.template .gitmessage
```

this command configures git to use this template only for this project, if you like to configure git to use it for all project you should add the global flag as follows:

```bash
git config --global commit.template ~/.gitmessage
```

When writing commit messages, please use the following conventions

* ADD adding new feature
* FIX a bug
* DOC documentation only
* REF refactoring that doesn't include any changes in features
* FMT formatting only (spacing...)
* MAK repository related changes (e.g., changes in the ignore list)
* TST related to test code only

Use bullet lists for commits including more than one change.
See the latest commit messages for an example before making your first commit!

### How to commit (a useful guide)

Commits should be **small** and related to a specific set of changes that are **semantically related to each other**.
Short commits are recommended to keep the repo clean and tidy: if you need to go back to a previous commit or making a-posteriori analyses of your code, finer granularity helps.

In case you need to make a big code refactoring, always remember that you can proceed by committing small incremental work that is still semantically self-contained.

Also, **think before committing!** You should design your commit before typing your `git commit` command, or even before modifying the code.
This helps you focusing on the function you are going to implement and better organize your work.

In case you did not think enough, and you made an unfortunate mistake, please read [this guide](https://sethrobertson.github.io/GitFixUm/fixup.html) before trying to solve the problem yourself and possibly stacking additional (unsolvable) mistakes.

## Useful links and materials

Please refer to [this guide](./MATERIALS.md) for useful links and materials.

## Debug

Please refer to the [debug file](./DEBUG.md) for more information about building and debugging ROS2 code by using VSCode.

## Authors

* **Enrico Ferrentino** - [UNISA Automatic Control Group](http://www.automatica.unisa.it/)

## Contributors

* **Federico Salvioli** - [ALTEC](https://www.altecspace.it/)

&copy; *2023 Automatic Control Group (UNISA)*
