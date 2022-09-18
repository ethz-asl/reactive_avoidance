# -*- coding: utf-8 -*-
"""Sphinx configuration file."""

import os
import time

import sphinx_rtd_theme
html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

# html_theme = "alabaster"
html_logo = "logo.png"


project = "rmpcpp"
master_doc = 'index'
author = "Michael Pantic"
copyright = "{}, {}".format(time.strftime("%Y"), author)
html_last_updated_fmt = "%c"
pygments_style = "sphinx"
templates_path = ["_templates"]
html_static_path = ['static']


extensions = [
     'breathe', 'exhale', 'sphinx.ext.autosectionlabel', 'recommonmark',
]

html_theme_options = {
    'canonical_url': 'https://ethz-asl.github.io/rmpcpp/rmpcpp/docs/html/',
    'analytics_id': 'UA-XXXXXXX-1',  #  Provided by Google in your dashboard
    'logo_only': True,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': True,
    'style_nav_header_background': 'green',
    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 3,
    # 'includehidden': True,
     'titles_only': True
}

html_context = {
    'display_github': True,
    'github_repo': 'rmpcpp',
    'github_user': 'ethz-asl',
    'github_version': 'develop-docs',
    'conf_py_path': '/docs/',
}

templates_path = [
    "_templates",
]


# Setup the breathe extension
breathe_projects = {"project": "./doxyoutput/xml"}
breathe_default_project = "project"

# Setup the exhale extension
exhale_args = {
    "verboseBuild": True,
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "Library API",
    "doxygenStripFromPath": "..",
    "createTreeView": True,
    "exhaleExecutesDoxygen": True,
    "exhaleUseDoxyfile": True,
    "pageLevelConfigMeta": ":github_url: https://github.com/ethz-asl/" + project
}
source_suffix = [ '.rst', '.md']

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'
