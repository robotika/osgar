How to contribute
=================

* master has its history never changed (no rebase) and always works
  - compiles
  - all tests pass
* all development is done on branches
* all branches are based on master and all PRs are to master
* branches and PRs are as contained and small as possible and reasonable
* each PR is reviewed by a member of @robotika
* before merging to master branches are squashed to cleanup the patchset to contain only relevant commits
* before merging the branch is rebased onto master to create a simple history (easy to revert and bisect)

