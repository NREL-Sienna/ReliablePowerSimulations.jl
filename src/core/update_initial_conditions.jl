function _make_initial_condition_outage_status(
    container,
    device::T,
    value,
    cache = nothing,
) where {T <: PSY.Component}
    return PSI.InitialCondition(device, _get_ref_outage_status(T, container), value, cache)
end

function _get_ref_outage_status(
    ::Type{T},
    container::PSI.InitialConditions,
) where {T <: PSY.Component}
    return PSI.UpdateRef{T}(OUTAGE, "outage")
end

function _get_outage_initial_value(
    d::PSY.Component,
    key::PSI.ICKey,
    ::PSI.AbstractDeviceFormulation,
    ::Nothing,
)
    return 0.0
end
