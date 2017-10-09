# -*- coding: utf-8 -*-
extensions = ['sphinx.ext.mathjax',
    'sphinx.ext.githubpages']

templates_path = ['_templates']
source_suffix = ['.rst', '.md']
master_doc = 'index'

project = u'EXOTica'
copyright = u'2017, Vladimir Ivan, Yiming Yang, Wolfgang Merkt, Michael Camilleri, Sethu Vijayakumar - and contributors'
author = u'Vladimir Ivan, Yiming Yang, Wolfgang Merkt, Michael Camilleri, Sethu Vijayakumar - and contributors'

# The short X.Y version.
version = u'5.0'
# The full version, including alpha/beta/rc tags.
release = u'beta1'

language = None
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
pygments_style = 'sphinx'
todo_include_todos = False


# -- Options for HTML output ----------------------------------------------
html_theme = 'alabaster'

html_context = {'show_source': False}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# This is required for the alabaster theme
# refs: http://alabaster.readthedocs.io/en/latest/installation.html#sidebars
html_sidebars = {
    '**': [
        'about.html',
        'navigation.html',
        'relations.html',  # needs 'show_related': True theme option to display
        'searchbox.html',
    ]
}


# -- Options for HTMLHelp output ------------------------------------------
htmlhelp_basename = 'EXOTicadoc'


# -- Options for LaTeX output ---------------------------------------------
latex_elements = {
    'papersize': 'a4paper',
    'pointsize': '11pt',
    'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'EXOTica.tex', u'EXOTica Documentation',
     u'Vladimir Ivan, Yiming Yang, Wolfgang Merkt, Michael Camilleri, Sethu Vijayakumar - and contributors', 'manual'),
]
