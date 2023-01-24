function PSI._get_initial_conditions_value(
    ::Vector{T},
    component::W,
    ::U,
    ::V,
    container::PSI.OptimizationContainer,
) where {
    T <: PSI.InitialCondition{InitialOutageStatus, Float64},
    V <: PSI.AbstractDeviceFormulation,
    W <: PSY.Component,
    U <: InitialOutageStatus,
}
    ic_data = PSI.get_initial_conditions_data(container)
    val = PSI.initial_condition_default(U(), component, V())
    @debug "Device $(PSY.get_name(component)) initialized DeviceStatus as $var_type" _group =
        PSI.LOG_GROUP_BUILD_INITIAL_CONDITIONS
    return T(component, val)
end

function PSI._get_initial_conditions_value(
    ::Vector{T},
    component::W,
    ::U,
    ::V,
    container::PSI.OptimizationContainer,
) where {
    T <: PSI.InitialCondition{InitialOutageStatus, PJ.ParameterRef},
    V <: PSI.AbstractDeviceFormulation,
    W <: PSY.Component,
    U <: InitialOutageStatus,
}
    ic_data = PSI.get_initial_conditions_data(container)
    val = PSI.initial_condition_default(U(), component, V())
    @debug "Device $(PSY.get_name(component)) initialized DeviceStatus as $var_type" _group =
        PSI.LOG_GROUP_BUILD_INITIAL_CONDITIONS
    return T(component, PSI.add_jump_parameter(PSI.get_jump_model(container), val))
end
