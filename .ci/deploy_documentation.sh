#!/bin/bash
# Legacy Travis CI deployment script — kept for reference.
# Active deployment is handled via .github/workflows/deploy_docs.yml (GitHub Actions).
#
# Build and deploy documentation
if [ "${ROS_DISTRO}" == "humble" ]; then
    if [ "${TRAVIS_BRANCH}" == "ros2" ]; then
        GIT_COMMIT=$(git rev-parse HEAD)
        echo "Building documentation for commit $GIT_COMMIT"
        /bin/bash ./.ci/build_documentation.sh > /dev/null 2>&1
        cd exotica/doc/_build/html
        git init
        git checkout --orphan gh-pages
        touch .nojekyll
        # Check if user configured email and name
        GIT_CONFIG_CHECK=$(git config --list --global)
        if [[ $GIT_CONFIG_CHECK != *"user.email"* ]]; then
            echo "Setting global git user settings"
            git config --global user.email "builds@travis-ci.com"
            git config --global user.name "Travis CI"
        fi
        git add --all > /dev/null 2>&1
        git commit -m "Update public documentation for $GIT_COMMIT"
        echo "Pushing public documentation to GitHub Pages"
        git push --quiet --force "https://$GITHUBKEY@github.com/ipab-slmc/exotica.git" gh-pages:gh-pages > /dev/null 2>&1
    else
        echo "Not ros2 branch, not building documentation."
    fi
fi
