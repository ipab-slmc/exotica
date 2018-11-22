#!/bin/bash
# Build and deploy documentation
# - cd $TRAVIS_BUILD_DIR
# - git config --global user.email "builds@travis-ci.com"
# - git config --global user.name "Travis CI"
# - bash .ci/build_documentation.sh > /dev/null 2>&1
# - git checkout --orphan gh-pages
# - mkdir old ; mv * old/ ; mv old/exotica/doc/_build/html/* . ; rm -rf old ; touch .nojekyll ; rm -rf .ci ; rm .clang-format .travis.yml .gitmodules .gitignore
# - git add --all
# - git commit -m "Update public documentation for $GIT_COMMIT"
# - git push --quiet --force https://$GITHUBKEY@github.com/ipab-slmc/exotica.git gh-pages:gh-pages > /dev/null 2>&1

if [ `lsb_release -c -s` = "xenial" ]; then
    if [ "${TRAVIS_BRANCH}" = "master" ]; then
        /bin/bash b./uild_documentation.sh > /dev/null 2>&1
        git checkout --orphan gh-pages
        mkdir old ; mv * old/ ; mv old/exotica/doc/_build/html/* . ; rm -rf old ; touch .nojekyll ; rm -rf .ci ; rm .clang-format .travis.yml .gitmodules .gitignore
        git add --all
    else
        echo "Not master, not building documentation."
    fi
fi
