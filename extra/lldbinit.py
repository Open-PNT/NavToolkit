import lldb
import os
import struct

err = lldb.SBError()
ELE_SIZE = 8
MAX_SUMMARY_SIZE = 3 * 3


def get_shape(valobj):
    type_name = valobj.GetTypeName()
    if 'Matrix3' in type_name:
        return (3, 3)
    if 'Vector3' in type_name:
        return (3, 1)
    if 'Vector4' in type_name:
        return (4, 1)
    shape = (
        valobj.GetChildMemberWithName('m_shape')
        .AddressOf()
        .GetValueAsUnsigned(0)
    )
    process = valobj.process
    rows = process.ReadUnsignedFromMemory(shape, 8, err)
    if err.Fail():
        return '<error: %s>' % str(err)
    cols = process.ReadUnsignedFromMemory(shape + 8, 8, err)
    if err.Fail():
        return '<error: %s>' % str(err)
    if rows == 0:
        rows = 1
    if cols == 0:
        cols = 1
    return (rows, cols)


def get_row_dynamic_tensor(process, dataPtr, cols):
    out = ""
    for c in range(cols):
        out += str(
            struct.unpack('d', process.ReadMemory(dataPtr + (c * 8), 8, err))[
                0
            ]
        )
        if c != cols - 1:
            out += ", "
    return out


def get_row_fixed_tensor(obj, row, cols):
    out = ""
    for c in range(cols):
        out += str(obj.GetChildAtIndex(c + row * cols).GetValue())
        if c < cols - 1:
            out += ', '
    return out


def MatrixSummary(valobj, stuff):
    process = valobj.process
    shape = get_shape(valobj)
    if isinstance(shape, str):
        return shape
    (rows, cols) = shape

    if rows * cols > MAX_SUMMARY_SIZE:
        return "Matrix ({}x{})".format(str(rows), str(cols))

    pointer_object = valobj.GetChildMemberWithName(
        'm_storage'
    ).GetChildMemberWithName('p_begin')
    if pointer_object.IsValid():
        dataPtr = pointer_object.GetValueAsUnsigned(0)

        if dataPtr == 0:
            return '<null buffer>'
        out = "[["
        for r in range(rows):
            offset = cols * ELE_SIZE * r
            out += get_row_dynamic_tensor(process, dataPtr + offset, cols)
            if r != rows - 1:
                out += "], ["
        return out + "]]"
    else:
        array = (
            valobj.GetChildMemberWithName('m_storage')
            .GetChildAtIndex(0)
            .GetChildAtIndex(0)
        )
        out = "[["
        for r in range(rows):
            out += get_row_fixed_tensor(array, r, cols)
            if r < rows - 1:
                out += "], ["
        return out + "]]"


class MatrixSyntheticChildrenProvider:
    valobj = None
    builtins = ["m_shape", "m_storage", "m_begin", "p_begin"]

    def __init__(self, valobj, internal_dict):
        self.valobj = valobj

    def num_children(self):
        shape = get_shape(self.valobj)
        if isinstance(shape, str):
            return None
        else:
            return len(self.builtins) + shape[0]

    def get_child_index(self, name):
        return self.builtins.index(name)

    def get_child_at_index(self, index):
        if index < len(self.builtins):
            return self.valobj.GetChildMemberWithName(self.builtins[index])
        else:
            shape = get_shape(self.valobj)
            if isinstance(shape, str):
                return None
            (rows, cols) = shape
            pointer_object = self.valobj.GetChildMemberWithName(
                'm_storage'
            ).GetChildMemberWithName('p_begin')
            if pointer_object.IsValid():
                dataPtr = pointer_object.GetValueAsUnsigned(0)
                if dataPtr == 0:
                    return None
                offset = cols * ELE_SIZE * (index - len(self.builtins))
                row = get_row_dynamic_tensor(
                    self.valobj.process, dataPtr + offset, cols
                )
                return self.valobj.CreateValueFromExpression(
                    "Row " + str(index - len(self.builtins)),
                    '(const char*)"' + row + '"',
                )
            else:
                array = (
                    self.valobj.GetChildMemberWithName('m_storage')
                    .GetChildAtIndex(0)
                    .GetChildAtIndex(0)
                )
                row_num = index - len(self.builtins)
                row = get_row_fixed_tensor(array, row_num, cols)
                return self.valobj.CreateValueFromExpression(
                    "Row " + str(row_num), '(const char*)"' + row + '"'
                )

    def update(self):
        pass

    def has_children(self):
        return not isinstance(get_shape(self.valobj), str)


def __lldb_init_module(debugger, internal_dict):
    matrix_types = [
        "xt::xtensor",
        "xt::xtensor_fixed",
        "navtk::Vector",
        "navtk::Vector3",
        "navtk::Vector4",
        "navtk::Matrix",
        "navtk::Matrix3",
    ]
    for matrix_type in matrix_types:
        debugger.HandleCommand(
            "type summary add %s -F %s.MatrixSummary"
            % (matrix_type, os.path.splitext(os.path.basename(__file__))[0])
        )
        debugger.HandleCommand(
            "type synthetic add %s --python-class %s.MatrixSyntheticChildrenProvider"
            % (matrix_type, os.path.splitext(os.path.basename(__file__))[0])
        )
