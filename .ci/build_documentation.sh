#!/bin/bash
cd exotica
doxygen
cd doc
make html || true
