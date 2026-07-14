# ///////////////////////////////////////////////////////////////
#
# BY: WANDERSON M.PIMENTA
# PROJECT MADE WITH: Qt Designer and PySide6
# V: 1.0.0
#
# This project can be used freely for all uses, as long as they maintain the
# respective credits only in the Python scripts, any information in the visual
# interface (GUI) can be modified without any implication.
#
# There are limitations on Qt licenses if you want to use your products
# commercially, I recommend reading them on the official website:
# https://doc.qt.io/qtforpython/licenses.html
#
# ///////////////////////////////////////////////////////////////

# Deliberately no imports here. Everything in this project imports submodules
# directly (``from modules.data_page import DataPage``), and keeping this
# initializer empty lets Qt-free submodules such as ``modules.sortie_analysis``
# be imported in headless environments (tests, scripts) where PySide6 or its
# system libraries (libGL) are unavailable.
