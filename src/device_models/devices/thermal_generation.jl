abstract type AbstractThermalOutageDispatchFormulation <:
              PSI.AbstractThermalDispatchFormulation end
abstract type AbstractThermalOutageCommitmentFormulation <:
              PSI.AbstractStandardUnitCommitment end
struct ThermalStandardUCOutages <: AbstractThermalOutageCommitmentFormulation end
struct ThermalBasicUCOutages <: AbstractThermalOutageCommitmentFormulation end
struct ThermalDispatchOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end

############## AuxiliaryOnVariable, ThermalGen ####################
PSI.get_variable_binary(::AuxiliaryOnVariable, ::Type{<:PSY.ThermalGen}, _) = true
# PSI.get_variable_initial_value(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) =
#     PSY.get_status(d) ? 1.0 : 0.0

PSI.get_variable_lower_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 1.0

PSI.get_variable_binary(::AuxiliaryStartVariable, ::Type{<:PSY.ThermalGen}, _) = true
PSI.get_variable_lower_bound(::AuxiliaryStartVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryStartVariable, d::PSY.ThermalGen, _) = 1.0

PSI.get_variable_binary(::AuxiliaryStopVariable, ::Type{<:PSY.ThermalGen}, _) = true
PSI.get_variable_lower_bound(::AuxiliaryStopVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryStopVariable, d::PSY.ThermalGen, _) = 1.0

######## CONSTRAINTS ############
PSI.initial_condition_default(
    ::InitialOutageStatus,
    d::PSY.ThermalGen,
    ::PSI.AbstractThermalFormulation,
) = PSY.get_status(d) ? 1 : 0
PSI.initial_condition_variable(
    ::InitialOutageStatus,
    d::PSY.ThermalGen,
    ::PSI.AbstractThermalFormulation,
) = PSI.OnVariable()

function _get_data_for_tdc(
    initial_conditions_on::Vector{T},
    initial_conditions_off::Vector{U},
    initial_conditions_outage::Vector{V},
    resolution::Dates.TimePeriod,
) where {T <: PSI.InitialCondition, U <: PSI.InitialCondition, V <: PSI.InitialCondition}
    steps_per_hour = 60 / Dates.value(Dates.Minute(resolution))
    fraction_of_hour = 1 / steps_per_hour
    lenght_devices_on = length(initial_conditions_on)
    lenght_devices_off = length(initial_conditions_off)
    lenght_devices_outage = length(initial_conditions_outage)

    IS.@assert_op lenght_devices_off == lenght_devices_on
    IS.@assert_op lenght_devices_off == lenght_devices_outage
    time_params = Vector{PSI.UpDown}(undef, lenght_devices_on)
    ini_conds = Matrix{PSI.InitialCondition}(undef, lenght_devices_on, 3)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_on)
        g = PSI.get_component(ic)
        IS.@assert_op g == PSI.get_component(initial_conditions_off[ix])
        time_limits = PSY.get_time_limits(g)
        name = PSY.get_name(g)
        if time_limits !== nothing
            if (time_limits.up <= fraction_of_hour) &&
               (time_limits.down <= fraction_of_hour)
                @debug "Generator $(name) has a nonbinding time limits. Constraints Skipped"
                continue
            else
                idx += 1
            end
            ini_conds[idx, 1] = ic
            ini_conds[idx, 2] = initial_conditions_off[ix]
            ini_conds[idx, 3] = initial_conditions_outage[ix]
            up_val = round(time_limits.up * steps_per_hour, RoundUp)
            down_val = round(time_limits.down * steps_per_hour, RoundUp)
            time_params[idx] = time_params[idx] = (up = up_val, down = down_val)
        end
    end
    if idx < lenght_devices_on
        ini_conds = ini_conds[1:idx, :]
        deleteat!(time_params, (idx + 1):lenght_devices_on)
    end
    return ini_conds, time_params
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{OutageTimeConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    ::PSI.DeviceModel{U, V},
    ::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    # Use getter functions that don't require creating the keys here
    time_steps = PSI.get_time_steps(container)
    device_names = [PSY.get_name(d) for d in devices]
    resolution = PSI.get_resolution(container)

    # Use getter functions that don't require creating the keys here
    initial_conditions_on =
        PSI.get_initial_condition(container, PSI.InitialTimeDurationOn(), U)
    initial_conditions_off =
        PSI.get_initial_condition(container, PSI.InitialTimeDurationOff(), U)
    initial_conditions_outage =
        PSI.get_initial_condition(container, InitialOutageStatus(), U)
    ini_conds, time_params = _get_data_for_tdc(
        initial_conditions_on,
        initial_conditions_off,
        initial_conditions_outage,
        resolution,
    )

    parameters = PSI.built_for_recurrent_solves(container)
    if !(isempty(ini_conds))
        if parameters
            device_duration_parameters_outage!(
                container,
                time_params,
                ini_conds,
                OutageTimeConstraint(),
                U,
            )
        else
            device_duration_look_ahead_outage!(
                container,
                time_params,
                ini_conds,
                OutageTimeConstraint(),
                U,
            )
        end
    end
    return
end

function _get_data_for_rocc_outage(
    container::PSI.OptimizationContainer,
    ::Type{T},
) where {T <: PSY.ThermalGen}
    resolution = PSI.get_resolution(container)
    if resolution > Dates.Minute(1)
        minutes_per_period = Dates.value(Dates.Minute(resolution))
    else
        @warn("Not all formulations support under 1-minute resolutions. Exercise caution.")
        minutes_per_period = Dates.value(Dates.Second(resolution)) / 60
    end

    initial_conditions_power = PSI.get_initial_condition(container, PSI.DevicePower(), T)
    initial_conditions_outage =
        PSI.get_initial_condition(container, InitialOutageStatus(), T)
    lenght_devices_power = length(initial_conditions_power)
    lenght_devices_outage = length(initial_conditions_outage)
    @assert lenght_devices_power == lenght_devices_outage
    data = Vector{DeviceOutageRampConstraintInfo}(undef, lenght_devices_power)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_power)
        g = ic.component
        @assert g == initial_conditions_outage[ix].component
        name = PSY.get_name(g)
        ramp_limits = PSY.get_ramp_limits(g)
        if !(ramp_limits === nothing)
            p_lims = PSY.get_active_power_limits(g)
            max_rate = abs(p_lims.min - p_lims.max) / minutes_per_period
            if (ramp_limits.up >= max_rate) & (ramp_limits.down >= max_rate)
                @debug "Generator $(name) has a nonbinding ramp limits. Constraints Skipped"
                continue
            else
                idx += 1
            end
            ramp = (
                up = ramp_limits.up * minutes_per_period,
                down = ramp_limits.down * minutes_per_period,
            )
            data[idx] = DeviceOutageRampConstraintInfo(
                name,
                p_lims,
                ic,
                initial_conditions_outage[ix],
                ramp,
            )
        end
    end
    if idx < lenght_devices_power
        deleteat!(data, (idx + 1):lenght_devices_power)
    end
    return data
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{OutageRampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::PSI.DeviceModel{U, V},
    W::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    add_semicontinuous_ramp_constraints_outages!(
        container,
        OutageRampConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        W,
    )
    return
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageRampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::PSI.DeviceModel{U, V},
    W::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: PSI.AbstractThermalDispatchFormulation}
    device_linear_rateofchange_outages!(
        container,
        T,
        PSI.ActivePowerVariable,
        devices,
        model,
        W,
    )
    return
end

function PSI.get_default_time_series_names(
    ::Type{<:PSY.ThermalGen},
    ::Type{
        <:Union{
            AbstractThermalOutageDispatchFormulation,
            AbstractThermalOutageCommitmentFormulation,
        },
    },
)
    return Dict{Type{<:PSI.TimeSeriesParameter}, String}(
        OutageTimeSeriesParameter => "outage",
    )
end

PSI.get_multiplier_value(
    ::PSI.TimeSeriesParameter,
    d::PSY.ThermalGen,
    ::AbstractThermalOutageDispatchFormulation,
) = PSY.get_max_active_power(d)
PSI.get_multiplier_value(
    ::PSI.TimeSeriesParameter,
    d::PSY.ThermalGen,
    ::AbstractThermalOutageCommitmentFormulation,
) = PSY.get_max_active_power(d)

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageCommitmentConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    device_outage_parameter!(container, T, devices, model, X)

    return
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: AbstractThermalOutageDispatchFormulation}
    device_outage_ub_parameter!(container, T, PSI.ActivePowerVariable, devices, model, X)
    return
end

function PSI.initial_conditions!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: AbstractThermalOutageDispatchFormulation}
    PSI.add_initial_condition!(container, devices, formulation, PSI.DevicePower())
    PSI.add_initial_condition!(container, devices, formulation, InitialOutageStatus())
    return
end

function PSI.initial_conditions!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: Union{ThermalStandardUCOutages}}
    PSI.add_initial_condition!(container, devices, formulation, PSI.DeviceStatus())
    PSI.add_initial_condition!(container, devices, formulation, PSI.DevicePower())
    PSI.add_initial_condition!(container, devices, formulation, PSI.InitialTimeDurationOn())
    PSI.add_initial_condition!(
        container,
        devices,
        formulation,
        PSI.InitialTimeDurationOff(),
    )
    PSI.add_initial_condition!(container, devices, formulation, InitialOutageStatus())
    return
end

function PSI.initial_conditions!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: Union{ThermalBasicUCOutages}}
    PSI.add_initial_condition!(container, devices, formulation, PSI.DeviceStatus())
    PSI.add_initial_condition!(container, devices, formulation, PSI.DevicePower())
    PSI.add_initial_condition!(container, devices, formulation, InitialOutageStatus())
    return
end

"""
Min and max active power limits of generators for thermal dispatch no minimum formulations
"""
function PSI.get_min_max_limits(
    device,
    ::Type{PSI.ActivePowerVariableLimitsConstraint},
    ::Type{S},
) where {S <: Union{ThermalNoMinOutages, ThermalNoMinRampLimitedOutages}}
    return (min = 0.0, max = PSY.get_active_power_limits(device).max)
end

PSI.get_variable_lower_bound(
    ::PSI.ActivePowerVariable,
    d::PSY.ThermalGen,
    ::Union{ThermalNoMinOutages, ThermalNoMinRampLimitedOutages},
) = 0.0

PSI._get_initial_condition_type(
    ::Type{OutageRampConstraint},
    ::Type{<:PSY.ThermalGen},
    ::Type{<:AbstractThermalOutageCommitmentFormulation},
) = PSI.DevicePower

PSI._get_initial_condition_type(
    ::Type{OutageRampConstraint},
    ::Type{<:PSY.ThermalGen},
    ::Type{<:AbstractThermalOutageDispatchFormulation},
) = PSI.DevicePower

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{<:PSI.PowerVariableLimitsConstraint},
    U::Type{<:Union{PSI.VariableType, PSI.ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: AbstractThermalOutageDispatchFormulation}
    if !has_semicontinuous_outage_feedforward(model, U)
        PSI.add_range_constraints!(container, T, U, devices, model, X)
    end
    return
end

# function PSI._add_pwl_term!(
#     container::PSI.OptimizationContainer,
#     component::T,
#     cost_data::Vector{PSY.VariableCost{Vector{Tuple{Float64, Float64}}}},
#     ::U,
#     ::V,
# ) where {T <: PSY.Component, U <: PSI.VariableType, V <: AbstractThermalOutageDispatchFormulation}
#     multiplier = PSI.objective_function_multiplier(U(), V())
#     resolution = PSI.get_resolution(container)
#     dt = Dates.value(Dates.Second(resolution)) / PSI.SECONDS_IN_HOUR
#     base_power = PSI.get_base_power(container)
#     # Re-scale breakpoints by Basepower
#     name = PSY.get_name(component)
#     time_steps = PSI.get_time_steps(container)
#     pwl_cost_expressions = Vector{JuMP.AffExpr}(undef, time_steps[end])
#     sos_val = PSI._get_sos_value(container, V, component)
#     for t in time_steps
#         data = PSY.get_cost(cost_data[t])
#         is_power_data_compact = PSI._check_pwl_compact_data(component, data, base_power)
#         if !PSI.uses_compact_power(component, V()) && is_power_data_compact
#             error(
#                 "The data provided is not compatible with formulation $V. Use a formulation compatible with Compact Cost Functions",
#             )
#             # data = _convert_to_full_variable_cost(data, component)
#         elseif PSI.uses_compact_power(component, V()) && !is_power_data_compact
#             data = PSI._convert_to_compact_variable_cost(data)
#         else
#             @debug PSI.uses_compact_power(component, V()) name T V
#             @debug is_power_data_compact name T V
#         end
#         slopes = PSY.get_slopes(data)
#         # First element of the return is the average cost at P_min.
#         # Shouldn't be passed for convexity check
#         is_convex = PSI._slope_convexity_check(slopes[2:end])
#         break_points = map(x -> last(x), data) ./ base_power
#         PSI._add_pwl_variables!(container, T, name, t, data)
#         _add_pwl_constraint_outages!(container, component, U(), break_points, sos_val, t)
#         if !is_convex
#             PSI._add_pwl_sos_constraint!(container, component, U(), break_points, sos_val, t)
#         end
#         pwl_cost = PSI._get_pwl_cost_expression(container, component, t, data, multiplier * dt)
#         pwl_cost_expressions[t] = pwl_cost
#     end
#     return pwl_cost_expressions
# end


function PSI._add_pwl_term!(
    container::PSI.OptimizationContainer,
    component::T,
    data::Vector{NTuple{2, Float64}},
    ::U,
    ::V,
) where {T <: PSY.Component, U <: PSI.VariableType, V <: AbstractThermalOutageDispatchFormulation}
    multiplier = PSI.objective_function_multiplier(U(), V())
    resolution = PSI.get_resolution(container)
    dt = Dates.value(Dates.Second(resolution)) / PSI.SECONDS_IN_HOUR
    base_power = PSI.get_base_power(container)
    # Re-scale breakpoints by Basepower
    name = PSY.get_name(component)

    is_power_data_compact = PSI._check_pwl_compact_data(component, data, base_power)

    if !PSI.uses_compact_power(component, V()) && is_power_data_compact
        error(
            "The data provided is not compatible with formulation $V. Use a formulation compatible with Compact Cost Functions",
        )
        # data = _convert_to_full_variable_cost(data, component)
    elseif PSI.uses_compact_power(component, V()) && !is_power_data_compact
        data = PSI._convert_to_compact_variable_cost(data)
    else
        @debug PSI.uses_compact_power(component, V()) name T V
        @debug is_power_data_compact name T V
    end

    slopes = PSY.get_slopes(data)
    # First element of the return is the average cost at P_min.
    # Shouldn't be passed for convexity check
    is_convex = PSI._slope_convexity_check(slopes[2:end])
    time_steps = PSI.get_time_steps(container)
    pwl_cost_expressions = Vector{JuMP.AffExpr}(undef, time_steps[end])
    break_points = map(x -> last(x), data) ./ base_power
    sos_val = PSI._get_sos_value(container, V, component)
    for t in time_steps
        PSI._add_pwl_variables!(container, T, name, t, data)
        _add_pwl_constraint_outages!(container, component, U(), break_points, sos_val, t)
        if !is_convex
            PSI._add_pwl_sos_constraint!(container, component, U(), break_points, sos_val, t)
        end
        pwl_cost = PSI._get_pwl_cost_expression(container, component, t, data, multiplier * dt)
        pwl_cost_expressions[t] = pwl_cost
    end
    return pwl_cost_expressions
end


function _add_pwl_constraint_outages!(
    container::PSI.OptimizationContainer,
    component::T,
    ::U,
    break_points::Vector{Float64},
    sos_status::PSI.SOSStatusVariable,
    period::Int,
) where {T <: PSY.Component, U <: PSI.VariableType}
    variables = PSI.get_variable(container, U(), T)
    const_container = PSI.lazy_container_addition!(
        container,
        PSI.PieceWiseLinearCostConstraint(),
        T,
        axes(variables)...,
    )
    len_cost_data = length(break_points)
    jump_model = PSI.get_jump_model(container)
    pwl_vars = PSI.get_variable(container, PSI.PieceWiseLinearCostVariable(), T)
    name = PSY.get_name(component)
    const_container[name, period] = JuMP.@constraint(
        jump_model,
        variables[name, period] ==
        sum(pwl_vars[name, ix, period] * break_points[ix] for ix in 1:len_cost_data)
    )

    if sos_status == PSI.SOSStatusVariable.NO_VARIABLE
        bin = 1.0
        @debug "Using Piecewise Linear cost function but no variable/parameter ref for ON status is passed. Default status will be set to online (1.0)" _group =
        PSI.LOG_GROUP_COST_FUNCTIONS

    elseif sos_status == PSI.SOSStatusVariable.PARAMETER
        bin = PSI.get_parameter(container, PSI.OnStatusParameter(), T).parameter_array[name, period]
        @debug "Using Piecewise Linear cost function with parameter OnStatusParameter, $T" _group =
        PSI.LOG_GROUP_COST_FUNCTIONS
    elseif sos_status == PSI.SOSStatusVariable.VARIABLE
        bin = PSI.get_variable(container, PSI.OnVariable(), T)[name, period]
        @debug "Using Piecewise Linear cost function with variable OnVariable $T" _group =
        PSI.LOG_GROUP_COST_FUNCTIONS
    else
        @assert false
    end

    JuMP.@constraint(
        jump_model,
        sum(pwl_vars[name, i, period] for i in 1:len_cost_data) <= bin
    )
    return
end