abstract type AbstractThermalOutageDispatchFormulation <: PSI.AbstractThermalDispatchFormulation end
struct ThermalStandardUCOutages <: PSI.AbstractStandardUnitCommitment end
struct ThermalBasicUCOutages <: PSI.AbstractStandardUnitCommitment end
struct ThermalDispatchOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinOutages <: AbstractThermalOutageDispatchFormulation end

############## AuxiliaryOnVariable, ThermalGen ####################
PSI.get_variable_binary(::AuxiliaryOnVariable, ::Type{<:PSY.ThermalGen}, _) = false
PSI.get_variable_initial_value(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = PSY.get_status(d) ? 1.0 : 0.0

PSI.get_variable_lower_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 1.0

######## CONSTRAINTS ############

function PSI.DeviceRangeConstraintSpec(
    ::Type{<:PSI.RangeConstraint},
    ::Type{<:PSI.VariableType},
    ::Type{T},
    ::Type{<:PSI.AbstractThermalFormulation},
    ::Type{<:PM.AbstractPowerModel},
    feedforward::SemiContinuousOutagesFF,
    use_parameters::Bool,
    use_forecasts::Bool,
) where {T <: PSY.ThermalGen}
    return PSI.DeviceRangeConstraintSpec()
end

function ramp_constraints!(
    optimization_container::PSI.OptimizationContainer,
    ::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    D <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages},
    S <: PSI.PM.AbstractPowerModel,
}
    data = PSI._get_data_for_rocc(optimization_container, T)
    if !isempty(data)
        # Here goes the reactive power ramp limits when versions for AC and DC are added
        for r in data
            PSI.add_device_services!(r, r.ic_power.device, model)
        end
        device_mixedinteger_rateofchange_outages!(
            optimization_container,
            data,
            PSI.make_constraint_name(PSI.RAMP, T),
            (
                PSI.make_variable_name(PSI.ActivePowerVariable, T),
                PSI.make_variable_name(PSI.StartVariable, T),
                PSI.make_variable_name(PSI.StopVariable, T),
            ),
            PSI.UpdateRef{T}(OUTAGE, "outage"),
        )
    else
        @warn "Data doesn't contain generators with ramp limits, consider adjusting your formulation"
    end
    return
end

function ramp_constraints!(
    optimization_container::PSI.OptimizationContainer,
    ::PSI.IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    D <: PSI.AbstractThermalDispatchFormulation,
    S <: PSI.PM.AbstractPowerModel,
}
    data = PSI._get_data_for_rocc(optimization_container, T)
    if !isempty(data)
        for r in data
            PSI.add_device_services!(r, r.ic_power.device, model)
        end
        # Here goes the reactive power ramp limits when versions for AC and DC are added
        device_linear_rateofchange_outages!(
            optimization_container,
            data,
            PSI.make_constraint_name(PSI.RAMP, T),
            PSI.make_variable_name(PSI.ActivePowerVariable, T),
            PSI.UpdateRef{T}(OUTAGE, "outage"),
        )
    else
        @warn "Data doesn't contain generators with ramp limits, consider adjusting your formulation"
    end
    return
end

function PSI.time_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, Union{ThermalStandardUCOutages, ThermalBasicUCOutages}},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    initial_conditions_on =
        PSI.get_initial_conditions(optimization_container, PSI.ICKey(PSI.InitialTimeDurationOn, T))
    initial_conditions_off = PSI.get_initial_conditions(
        optimization_container,
        PSI.ICKey(PSI.InitialTimeDurationOff, T),
    )
    ini_conds, time_params =
        PSI._get_data_for_tdc(initial_conditions_on, initial_conditions_off, resolution)
    forecast_label = "outage"
    constraint_infos = Vector{DeviceDurationConstraintInfo}()
    for (ix, ic) in enumerate(ini_conds[:, 1])
        name = PSI.get_name(ic.device)
        info = DeviceDurationConstraintInfo(
            name,
            time_params[ix],
            Tuple(ini_conds[ix, :]),
            1.0,
            PSI.get_time_series(optimization_container, ic.device, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(ini_conds))
        if parameters
            device_duration_parameters_outage!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(PSI.DURATION, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                ),
                PSI.UpdateRef{T}(OUTAGE, forecast_label),
            )
        else
            device_duration_look_ahead_outage!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(PSI.DURATION, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                ),
            )
        end
    else
        @warn "Data doesn't contain generators with time-up/down limits, consider adjusting your formulation"
    end
    return
end

function outage_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel, D <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    # initial_conditions =
    #     PSI.get_initial_conditions(optimization_container, PSI.ICKey(InitialOutageStatus, T))
    forecast_label = "outage"
    constraint_infos = Vector{DeviceOutageConstraintInfo}()
    for ic in devices
        name = PSI.get_name(ic)
        info = DeviceOutageConstraintInfo(
            name,
            nothing, #ic,
            1.0,
            PSI.get_time_series(optimization_container, ic, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(devices))
        if parameters
            device_outage_parameter!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                ),
                PSI.UpdateRef{T}(OUTAGE, forecast_label),
            )
        else
            device_outage!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                ),
            )
        end
    else
        @warn "Data doesn't contain generators with initial condition for outage status, consider adjusting your formulation"
    end

    return
end

function outage_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel, D <: ThermalDispatchOutages}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    # initial_conditions =
    #     PSI.get_initial_conditions(optimization_container, PSI.ICKey(InitialOutageStatus, T))
    forecast_label = "outage"
    constraint_infos = Vector{DeviceOutageConstraintInfo}()
    for ic in devices
        name = PSI.get_name(ic)
        info = DeviceOutageConstraintInfo(
            name,
            nothing, # ic,
            1.0,
            PSI.get_time_series(optimization_container, ic, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(devices))
        if parameters
            device_outage_ub_parameter!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                PSI.make_variable_name(PSI.ACTIVE_POWER, T),
                PSI.UpdateRef{T}(OUTAGE, forecast_label),
            )
        else
            device_outage_ub!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                PSI.make_variable_name(PSI.ACTIVE_POWER, T),
            )
        end
    else
        @warn "Data doesn't contain generators with initial condition for outage status, consider adjusting your formulation"
    end

    return
end


function add_outage_parameter!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, D},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel, D <: ThermalDispatchOutages}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    time_steps = PSI.model_time_steps(optimization_container)
    set_names = [PSY.get_name(d) for d in devices]
    forecast_label = "outage"
    container_outage = PSI.add_param_container!(
        optimization_container,
        PSI.UpdateRef{T}(OUTAGE, forecast_label),
        set_names,
        time_steps,
    )
    param = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)
    
    for d in devices, t in time_steps
        name = PSY.get_name(d)
        ts_vector = PSI.get_time_series(optimization_container, d, forecast_label)
        param[name, t] =
            PJ.add_parameter(optimization_container.JuMPmodel, ts_vector[t])
        multiplier[name, t] = 1.0
    end
    return
end


function PSI.initial_conditions!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    PSI.status_initial_condition!(optimization_container, devices, formulation)
    PSI.output_initial_condition!(optimization_container, devices, formulation)
    PSI.duration_initial_condition!(optimization_container, devices, formulation)
    # outage_status_initial_condition!(optimization_container, devices, formulation)
    return
end

function outage_status_initial_condition!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    ::D,
) where {T <: PSY.ThermalGen, D <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    PSI._make_initial_conditions!(
        optimization_container,
        devices,
        D(),
        nothing,
        PSI.ICKey(InitialOutageStatus, T),
        _make_initial_condition_outage_status,
        _get_outage_initial_value,
    )

    return
end
