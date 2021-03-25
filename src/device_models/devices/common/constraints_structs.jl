struct DeviceDurationConstraintInfo <: PSI.AbstractStartConstraintInfo
    component_name::String
    duration_data::PSI.UpDown
    initial_duration::Tuple{PSI.InitialCondition}
    multiplier::Float64
    timeseries::Vector{Float64}
end

#TODO: Constraint struct outage start/stop 
#=
struct  <: PSI.AbstractStartConstraintInfo
    component_name::String
    multiplier::Float64
    timeseries::Vector{Float64}
end
=#
