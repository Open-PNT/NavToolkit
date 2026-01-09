# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))

import os
import subprocess
import json

# -- Project information -----------------------------------------------------

project = 'NavToolkit'
copyright = '2019-2025, IS4S'
author = 'IS4S'

# The full version, including alpha/beta/rc tags
projectinfo = json.loads(
    subprocess.check_output(
        ["meson", "introspect", "--projectinfo", "../meson.build"]
    )
)
__version__ = projectinfo['version']

release = __version__
version = __version__

# Enable numref
numfig = True

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['breathe', 'sphinx.ext.mathjax']

if 'DOC_PREVIEW' not in os.environ or os.environ['DOC_PREVIEW'] != '1':
    extensions.append('exhale')

# Relative to _static directory
mathjax_path = 'es5/tex-mml-chtml.js'

mathjax3_config = {
    'TeX': {'equationNumbers': {'autoNumber': 'all', 'useLabelIds': True}},
    'tex2jax': {
        'inlineMath': [['$', '$'], ['\\(', '\\)']],
        'displayMath': [['$$', '$$'], ['\\[', '\\]']],
        'processEscapes': 'true',
    },
}


breathe_projects = {"navtk": "doxygen_output/xml"}

breathe_default_project = "navtk"

exhale_args = {
    # These arguments are required
    "containmentFolder": "./api-exhale",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "NavToolkit API",
    "doxygenStripFromPath": "..",
    # Suggested optional arguments
    "createTreeView": True,
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['Thumbs.db', '.DS_Store', 'resources']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static', '../subprojects/MathJax']

# Add :orphan: to the top of the .rst files so they aren't expected in the TOC
rst_prolog = """
:orphan:
"""
