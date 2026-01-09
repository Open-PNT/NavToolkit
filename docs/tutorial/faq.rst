.. _rst_faq:

Frequently Asked Questions
==========================

What's the difference between NavToolkit and AFIT's Scorpion Software Suite?
-------------------------------------------------------------------------------

The AFIT Scorpion Software Suite was initially designed to support academic
research use cases and was built to run on the Java Virtual Machine (JVM).  It
is written primarily in Kotlin. NavToolkit (navtk) is a rewrite of the
Scorpion Software Suite in C++, designed to support both operational and
research use cases.

What's the difference between pntOS, Viper Plugins, and NavToolkit?
----------------------------------------------------------------------

pntOS is the specification of a modular plug-in architecture for building a PNT
sensor fusion solution that is able to ingest GPS and complementary navigation
signals.

Viper Plugins is the name of a government-owned reference implementation of a
set of plugins that implement the pntOS specification.

NavToolkit (navtk) is a software library that contains navigation
algorithms used in the implementation of the Viper plugins. Much of Viper
Plugins will be built using NavToolkit but anyone is free to develop plugins
using their own internal software libraries.

What Platforms Does NavToolkit Support?
---------------------------------------------

The reference implementation of NavToolkit is written in C++. In addition,
we have bindings that support usage of the compiled NavToolkit binary from
Python by importing a subset of the classes and functions from a shared
library. This document contains examples of how to use NavToolkit from both
C++ and Python.

NavToolkit supports building on bare-metal on macOS, Ubuntu, and Fedora.
We have received user reports that NavToolkit is buildable on other Linux
distributions and on Windows. However, we do not actively test or guarantee
functionality on those configurations. A few additional targets are built in
docker or are cross-compiled.

How Do I Modify an Existing C++ StateBlock / MeasurementProcessor from Python?
------------------------------------------------------------------------------

Due to technical limitations, when writing Python code it is only possible to
implement a new class. As a workaround, you can use delegation of
implementation. For example, suppose you wanted to modify the
:class:`~navtk::filtering::Pinson15NedBlock` to add an additional state. Then
you could:

1. Define a new standalone block called ``My16StateBlock`` as you normally
   would.
2. When you implement your
   :func:`~navtk::filtering::StateBlock::generate_dynamics` function, you
   internally make a copy of :class:`~navtk::filtering::Pinson15NedBlock` and
   call :func:`~navtk::filtering::StateBlock::generate_dynamics` on the Pinson
   block to get the 15-state matrices from it. Then construct your `16x16`
   matrix, copy the 15 states from Pinson, and set the 16th state manually.
3. Similarly delegate any of the other methods to the internal Pinson block
   when possible.
