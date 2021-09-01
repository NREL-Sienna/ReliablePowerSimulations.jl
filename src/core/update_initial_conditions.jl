function PSI.calculate_ic_quantity(
    ::PSI.ICKey{InitialOutageStatus, T},
    ic::PSI.InitialCondition,
    var_value::Float64,
    simulation_cache::Dict{<:PSI.CacheKey, PSI.AbstractCache},
    elapsed_period::Dates.Period,
) where {T <: PSY.Component}
    # cache = PSI.get_cache(simulation_cache, ic.cache_type, T)
    # name = PSI.get_device_name(ic)
    # time_cache = PSI.cache_value(cache, name)

    # current_counter = time_cache[:count]
    # last_status = time_cache[:status]
    # var_status = isapprox(var_value, 0.0, atol = ABSOLUTE_TOLERANCE) ? 0.0 : 1.0
    # # @debug last_status, var_status, abs(last_status - var_status)
    # @assert abs(last_status - var_status) < ABSOLUTE_TOLERANCE

    return 1.0
end

function _make_initial_condition_outage_status(
    container,
    device::T,
    value,
    cache = nothing,
) where {T <: PSY.Component}
    return PSI.InitialCondition(device, PSI._get_ref_active_power(T, container), value, cache)
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
    return 1.0
end
