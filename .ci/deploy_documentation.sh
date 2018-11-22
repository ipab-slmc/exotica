#!/bin/bash
# Build and deploy documentation
set -e
if [ `lsb_release -c -s` = "xenial" ]; then
    if [ "${TRAVIS_BRANCH}" = "master" ]; then
        GIT_COMMIT = `git rev-parse HEAD`
        /bin/bash ./build_documentation.sh > /dev/null 2>&1
        git checkout --orphan gh-pages
        mkdir old ; mv * old/ ; mv old/exotica/doc/_build/html/* . ; rm -rf old ; touch .nojekyll ; rm -rf .ci ; rm .clang-format .travis.yml .gitmodules .gitignore
        # Check if user configured email and name
        GIT_CONFIG_CHECK = `git config --list --global`
        if [[ $GIT_CONFIG_CHECK != *"user.email"* ]]; then
            git config --global user.email "builds@travis-ci.com"
            git config --global user.name "Travis CI"
        fi
        git add --all
        git commit -m "Update public documentation for $GIT_COMMIT"
        git push --quiet --force https://$GITHUBKEY@github.com/ipab-slmc/exotica.git gh-pages:gh-pages > /dev/null 2>&1
    else
        echo "Not master, not building documentation."
    fi
fi
