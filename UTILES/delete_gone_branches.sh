#!/bin/bash

git fetch --prune

for branch in $(git branch -vv | awk '/: gone]/{print $1}'); do
    echo "Deleting $branch"
    git branch -d "$branch"
done
