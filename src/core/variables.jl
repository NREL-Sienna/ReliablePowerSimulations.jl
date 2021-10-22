
const OUTAGE = "outage"
const AUXILIARY_ON = "Aux_On"

struct AuxiliaryOnVariable <: PSI.VariableType end

PSI.make_variable_name(::Type{AuxiliaryOnVariable}, ::Type{T}) where {T <: PSY.Device} =
    PSI.encode_symbol(T, "Aux_On")
