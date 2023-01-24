struct BookKeepingwReservationOutage <: PSI.AbstractStorageFormulation end
struct EndOfPeriodEnergyTargetOutage <: PSI.AbstractEnergyManagement end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.Storage, W <: PSI.AbstractStorageFormulation}
    device_outage_ub_parameter!(
        container,
        T,
        PSI.ActivePowerInVariable,
        devices, 
        model,
        X,
    )
    device_outage_ub_parameter!(
        container,
        T,
        PSI.ActivePowerOutVariable,
        devices, 
        model,
        X,
    )
    return
end
