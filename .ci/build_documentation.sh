#!/bin/bash
cd exotica/doc
doxygen
make html || true
touch _build/html/.nojekyll
