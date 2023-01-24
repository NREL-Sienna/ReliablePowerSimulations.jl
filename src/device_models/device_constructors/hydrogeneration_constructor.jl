function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{H, D},
    ::Type{S},
) where {H <: PSY.HydroGen, D <: HydroDispatchRunOfRiverOutage, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(H, sys)

    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, D())
    PSI.add_variables!(container, PSI.ReactivePowerVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyOutput, devices, D())
    # PSI.add_variables!(container, OutageVariable, devices, D())
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ReactivePowerBalance,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_parameters!(container, PSI.ActivePowerTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerRangeExpressionLB,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerRangeExpressionUB,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_expressions!(container, PSI.ProductionCostExpression, devices, model)

    PSI.add_feedforward_arguments!(container, model, devices)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{H, D},
    ::Type{S},
) where {H <: PSY.HydroGen, D <: HydroDispatchRunOfRiverOutage, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(H, sys)

    PSI.add_constraints!(
        container,
        PSI.ActivePowerVariableLimitsConstraint,
        PSI.ActivePowerRangeExpressionLB,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.ActivePowerVariableLimitsConstraint,
        PSI.ActivePowerRangeExpressionUB,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.ReactivePowerVariableLimitsConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        OutageUpperBoundConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.objective_function!(container, devices, model, S)

    PSI.add_constraint_dual!(container, sys, model)
    return
end

"""
Construct model for HydroGen with RunOfRiver Dispatch Formulation
with only Active Power.
"""
function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{H, D},
    ::Type{S},
) where {
    H <: PSY.HydroGen,
    D <: HydroDispatchRunOfRiverOutage,
    S <: PM.AbstractActivePowerModel,
}
    devices = PSI.get_available_components(H, sys)

    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyOutput, devices, D())
    # PSI.add_variables!(container, OutageVariable, devices, D())
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_parameters!(container, PSI.ActivePowerTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerRangeExpressionLB,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerRangeExpressionUB,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_expressions!(container, PSI.ProductionCostExpression, devices, model)

    PSI.add_feedforward_arguments!(container, model, devices)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{H, D},
    ::Type{S},
) where {
    H <: PSY.HydroGen,
    D <: HydroDispatchRunOfRiverOutage,
    S <: PM.AbstractActivePowerModel,
}
    devices = PSI.get_available_components(H, sys)

    PSI.add_constraints!(
        container,
        PSI.ActivePowerVariableLimitsConstraint,
        PSI.ActivePowerRangeExpressionLB,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.ActivePowerVariableLimitsConstraint,
        PSI.ActivePowerRangeExpressionUB,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        OutageUpperBoundConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.objective_function!(container, devices, model, S)

    PSI.add_constraint_dual!(container, sys, model)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{H, D},
    ::Type{S},
) where {
    H <: PSY.HydroEnergyReservoir,
    D <: HydroDispatchReservoirStorageOutage,
    S <: PM.AbstractPowerModel,
}
    devices = PSI.get_available_components(H, sys)

    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, D())
    PSI.add_variables!(container, PSI.ReactivePowerVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyVariable, devices, D())
    # PSI.add_variables!(container, OutageVariable, devices, D())
    PSI.add_variables!(container, PSI.WaterSpillageVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyShortageVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergySurplusVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyOutput, devices, D())
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ReactivePowerBalance,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_parameters!(container, PSI.EnergyTargetTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, PSI.InflowTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)

    PSI.add_expressions!(container, PSI.ProductionCostExpression, devices, model)

    PSI.add_to_expression!(
        container,
        PSI.ActivePowerRangeExpressionLB,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerRangeExpressionUB,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_feedforward_arguments!(container, model, devices)

    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{H, HydroDispatchReservoirStorageOutage},
    ::Type{S},
) where {H <: PSY.HydroEnergyReservoir, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(H, sys)

    PSI.add_constraints!(
        container,
        PSI.ActivePowerVariableLimitsConstraint,
        PSI.ActivePowerRangeExpressionLB,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.ActivePowerVariableLimitsConstraint,
        PSI.ActivePowerRangeExpressionUB,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.ReactivePowerVariableLimitsConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_initial_condition!(
        container,
        devices,
        HydroDispatchReservoirStorageOutage(),
        PSI.InitialEnergyLevel(),
    )
    # Energy Balance Constraint
    PSI.add_constraints!(container, PSI.EnergyBalanceConstraint, devices, model, S)
    PSI.add_constraints!(container, PSI.EnergyTargetConstraint, devices, model, S)
    PSI.add_constraints!(
        container,
        OutageUpperBoundConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.objective_function!(container, devices, model, S)
    PSI.add_constraint_dual!(container, sys, model)

    return
end

"""
Construct model for HydroGen with ReservoirStorage Dispatch Formulation
with only Active Power
"""
function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{H, D},
    ::Type{S},
) where {
    H <: PSY.HydroEnergyReservoir,
    D <: HydroDispatchReservoirStorageOutage,
    S <: PM.AbstractActivePowerModel,
}
    devices = PSI.get_available_components(H, sys)

    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyVariable, devices, D())
    # PSI.add_variables!(container, OutageVariable, devices, D())
    PSI.add_variables!(container, PSI.WaterSpillageVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyShortageVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergySurplusVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyOutput, devices, D())
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_parameters!(container, PSI.EnergyTargetTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, PSI.InflowTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)

    PSI.add_expressions!(container, PSI.ProductionCostExpression, devices, model)

    PSI.add_to_expression!(
        container,
        PSI.ActivePowerRangeExpressionLB,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerRangeExpressionUB,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_feedforward_arguments!(container, model, devices)

    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{H, HydroDispatchReservoirStorageOutage},
    ::Type{S},
) where {H <: PSY.HydroEnergyReservoir, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(H, sys)

    PSI.add_constraints!(
        container,
        PSI.ActivePowerVariableLimitsConstraint,
        PSI.ActivePowerRangeExpressionLB,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.ActivePowerVariableLimitsConstraint,
        PSI.ActivePowerRangeExpressionUB,
        devices,
        model,
        S,
    )

    PSI.add_initial_condition!(
        container,
        devices,
        HydroDispatchReservoirStorageOutage(),
        PSI.InitialEnergyLevel(),
    )
    # Energy Balance Constraint
    PSI.add_constraints!(container, PSI.EnergyBalanceConstraint, devices, model, S)
    PSI.add_constraints!(container, PSI.EnergyTargetConstraint, devices, model, S)
    PSI.add_constraints!(
        container,
        OutageUpperBoundConstraint,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.objective_function!(container, devices, model, S)
    PSI.add_constraint_dual!(container, sys, model)

    return
end

"""
Construct model for HydroPumpedStorage with PumpedStorage Dispatch Formulation
with only Active Power
"""
function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{H, D},
    ::Type{S},
) where {
    H <: PSY.HydroPumpedStorage,
    D <: HydroDispatchPumpedStoragewReservationOutage,
    S <: PM.AbstractActivePowerModel,
}
    devices = PSI.get_available_components(H, sys)

    PSI.add_variables!(container, PSI.ActivePowerInVariable, devices, D())
    PSI.add_variables!(container, PSI.ActivePowerOutVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyVariableUp, devices, D())
    PSI.add_variables!(container, PSI.EnergyVariableDown, devices, D())
    PSI.add_variables!(container, PSI.WaterSpillageVariable, devices, D())
    PSI.add_variables!(container, PSI.EnergyOutput, devices, D())
    # PSI.add_variables!(container, OutageVariable, devices, D())
    if PSI.get_attribute(model, "reservation")
        PSI.add_variables!(container, PSI.ReservationVariable, devices, D())
    end

    PSI.add_parameters!(container, PSI.InflowTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, PSI.OutflowTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)

    PSI.add_expressions!(container, PSI.ProductionCostExpression, devices, model)

    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
    )

    PSI.add_expressions!(container, PSI.ReserveRangeExpressionLB, devices, model)
    PSI.add_expressions!(container, PSI.ReserveRangeExpressionUB, devices, model)

    PSI.add_feedforward_arguments!(container, model, devices)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{H, HydroDispatchPumpedStoragewReservationOutage},
    ::Type{S},
) where {H <: PSY.HydroPumpedStorage, S <: PM.AbstractActivePowerModel}
    devices = PSI.get_available_components(H, sys)

    PSI.add_constraints!(
        container,
        PSI.OutputActivePowerVariableLimitsConstraint,
        PSI.ActivePowerOutVariable,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(
        container,
        PSI.InputActivePowerVariableLimitsConstraint,
        PSI.ActivePowerInVariable,
        devices,
        model,
        S,
    )

    PSI.add_initial_condition!(
        container,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
        PSI.InitialEnergyLevelUp(),
    )
    PSI.add_initial_condition!(
        container,
        devices,
        HydroDispatchPumpedStoragewReservationOutage(),
        PSI.InitialEnergyLevelDown(),
    )

    # Energy Balanace limits
    PSI.add_constraints!(container, PSI.EnergyCapacityUpConstraint, devices, model, S)
    PSI.add_constraints!(container, PSI.EnergyCapacityDownConstraint, devices, model, S)
    PSI.add_constraints!(container, OutageUpperBoundConstraint, devices, model, S)

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.objective_function!(container, devices, model, S)

    PSI.add_constraint_dual!(container, sys, model)
    return
end
