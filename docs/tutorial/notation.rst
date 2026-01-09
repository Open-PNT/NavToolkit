.. _rst_notation:

Notation
============

Unless otherwise specified (explicitly or by context), the following notation
conventions are used in this documentation:

- **Scalar** - Italic, normally lower case.  Examples: :math:`x`, :math:`\phi`
  
- **Vector** - Bold, lower-case, non-italic for alpha-numeric symbols.  
  Examples: :math:`\mathbf{x}`, :math:`\pmb{\phi}`
  
- **Matrix** - Bold, upper-case letters and symbols, non-italic.  Examples: 
  :math:`\mathbf{A}`, :math:`\mathbf{P}`, :math:`\mathbf{\Phi}`
  
- **Estimated Value** - Caret (hat) above the variable.  Examples:
  :math:`\hat{x}` (scalar), :math:`\hat{\mathbf{x}}` (vector)
  
- **Coordinate Frames** - Represented by non-italic superscript or subscript.  
  Examples: :math:`\mathbf{f}^\mathrm{ECEF}`, :math:`\mathbf{r}^\mathrm{NED}`
  
- **Coordinate Frame Conversion** - Coordinate frame being converted *from* as
  subscript non-italic 
  font and the coordinate frame being converted *to* as superscript 
  non-italic font.  Examples: :math:`\mathbf{C}_\mathrm{N}^\mathrm{ENU}` 
  is a DCM that describes a rotation which transforms a vector from the 
  :math:`\mathrm{N}` frame to the :math:`\mathrm{ENU}` 
  frame, :math:`\mathbf{q}_\mathrm{ECEF}^\mathrm{NED}` is a quaternion that 
  describes a rotation which transforms a vector from the :math:`\mathrm{ECEF}`
  frame to the :math:`\mathrm{NED}` frame.  
  See `Coordinate Frame Definitions <./coordinate_frames.html>`_ 
  for more detail.
  