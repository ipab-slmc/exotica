#!/bin/bash
cd exotica/doc
doxygen
make html || true
cd ../..
