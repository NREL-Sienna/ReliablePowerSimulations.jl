function PSI._get_initial_conditions_value(
    ::Vector{T},
    component::W,
    ::U,
    ::V,
    container::PSI.OptimizationContainer,
) where {
    T <: PSI.InitialCondition{U, Float64},
    V <: PSI.AbstractDeviceFormulation,
    W <: PSY.Component,
} where {U <: InitialOutageStatus}
    ## Assume all generators are available at the start of the simulation
    val = 1.0
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
    T <: PSI.InitialCondition{U, JuMP.VariableRef},
    V <: PSI.AbstractDeviceFormulation,
    W <: PSY.Component,
    U <: InitialOutageStatus,
}
    ## Assume all generators are available at the start of the simulation
    val = 1.0
    @debug "Device $(PSY.get_name(component)) initialized DeviceStatus as $var_type" _group =
        PSI.LOG_GROUP_BUILD_INITIAL_CONDITIONS
    return T(component, PSI.add_jump_parameter(PSI.get_jump_model(container), val))
end


function PSI.get_system_state_value(
    state::PSI.SimulationState,
    ::T,
    ::Type{U},
) where {T <: PSI.ParameterType, U <: Union{PSY.Component, PSY.System}}
    return PSI.get_system_state_value(state, PSI.ConstraintKey(T, U))
end

function PSI.update_initial_conditions!(
    ics::Vector{T},
    state::PSI.SimulationState,
    ::Dates.Millisecond,
) where {
    T <: PSI.InitialCondition{InitialOutageStatus, S},
} where {S <: Union{Float64, JuMP.VariableRef}}
    for ic in ics
        var_val = PSI.get_system_state_value(
            state,
            OutageTimeSeriesParameter(),
            PSI.get_component_type(ic),
        )
        PSI.set_ic_quantity!(ic, var_val[PSI.get_component_name(ic)])
    end
    return
end

function PSI.update_initial_conditions!(
    ics::Vector{T},
    state::PSI.SimulationState,
    ::Dates.Millisecond,
) where {
    T <: PSI.InitialCondition{PSI.DeviceStatus, S},
} where {S <: Union{Float64, JuMP.VariableRef}}
    for ic in ics
        var_val = PSI.get_system_state_value(
            state,
            DeviceStatus(),
            PSI.get_component_type(ic),
        )
        PSI.set_ic_quantity!(ic, var_val[PSI.get_component_name(ic)])
    end
    return
end
