# -*- coding: utf-8 -*-
extensions = ['sphinx.ext.mathjax',
              #'sphinx.ext.githubpages', # creates .nojekyll on generated directory, only in >1.4
              'sphinx.ext.autodoc',
              'sphinx.ext.intersphinx',
              'sphinx.ext.autosummary',
              'sphinx.ext.autosectionlabel',
              'sphinx.ext.todo',
              'sphinx.ext.napoleon']

autosummary_generate = True

templates_path = ['_templates']
source_suffix = ['.rst', '.md']
master_doc = 'index'

project = u'EXOTica'
copyright = u'2020, Vladimir Ivan, Wolfgang Merkt, Sethu Vijayakumar - and contributors'
author = u'Vladimir Ivan, Wolfgang Merkt, Sethu Vijayakumar - and contributors'

# The short X.Y version.
version = u'6.0.2'
# The full version, including alpha/beta/rc tags.
release = u'6.0.2'

language = None
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'Doxyfile']
pygments_style = 'sphinx'
todo_include_todos = False


# -- Options for HTML output ----------------------------------------------
html_theme = 'sphinx_rtd_theme'
# html_logo = 'images/EXOTica_logo.png'
html_context = {'show_source': False}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ['_static']

html_extra_path = ['doxygen_cpp']


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
