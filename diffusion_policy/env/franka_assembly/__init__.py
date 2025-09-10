from gym.envs.registration import register
import diffusion_policy.env.franka_assembly

register(
    id='FrankaAssembly-v0',
    entry_point='diffusion_policy.env.franka_assembly.franka_assembly_env:FrankaAssemblyEnv',
    max_episode_steps=300,
)