.. _rst_adding_support_python:

Adding State/Sensor Support (Python)
====================================

While it is possible to add support for new states and measurements directly
within the NavToolkit C++ source code, sometimes it is more convenient to
develop such modules separately in Python. NavToolkit includes Python
bindings for many of its classes.

Custom Python Classes
-------------------------

NavToolkit allows Python classes to inherit from several C++ classes.

To add support for a new sensor, the user can define a new
:class:`~navtk::filtering::StateBlock` to add new states to the filter, and
a new :class:`~navtk::filtering::MeasurementProcessor` to support the 
measurement inputs. A new :class:`~navtk::filtering::StateBlock` must implement
the methods that describe a set of one or more states and how they propagate
through time, while a new :class:`~navtk::filtering::MeasurementProcessor` must
implement the methods that describe how a measurement relates to one or more
:class:`~navtk::filtering::StateBlock`\s.

Creating a Python StateBlock
--------------------------------

In order to create the Python equivalent to the ``BiasBlock.hpp`` we created in
:ref:`rst_adding_support_cpp`, ``BiasBlock.py``, we'll write:

.. literalinclude:: ../../examples/bias_with_update/BiasBlock.py
   :language: python

Creating a Python MeasurementProcessor
-------------------------------------------

Note that the location of the NavToolkit shared library needs to be added to
the path in order to import NavToolkit modules. In this case we're assuming the
library resides in the subdirectory ``build``.

So far we've created a new :class:`~navtk::filtering::StateBlock` that can be
added to a fusion engine like any other :class:`~navtk::filtering::StateBlock`
can. The next step is to implement a
:class:`~navtk::filtering::MeasurementProcessor` which uses our new
:class:`~navtk::filtering::StateBlock`. We can look at the interface for
:class:`~navtk::filtering::MeasurementProcessor` and follow similar steps as in
:ref:`rst_adding_support_cpp` to create ``BiasMeasurementProcessor.py``:

.. literalinclude:: ../../examples/bias_with_update/BiasMeasurementProcessor.py
   :language: python

Using the StateBlock and MeasurementProcessor with a Fusion Engine
---------------------------------------------------------------------

Then we can create a script which uses both the
:class:`~navtk::filtering::StateBlock` and the
:class:`~navtk::filtering::MeasurementProcessor` in a fusion engine. We'll
create ``bias_example_with_update.py``:

.. literalinclude:: ../../examples/bias_with_update/bias_example_with_update.py
   :language: python

If you'd like to try this example out you can find it (as well as the
:class:`~navtk::filtering::StateBlock` and
:class:`~navtk::filtering::MeasurementProcessor`) in the
``examples/bias_with_update`` directory. It can be run from the NavToolkit root
folder via the command:

.. code-block:: bash

   examples/bias_with_update/bias_example_with_update.py
