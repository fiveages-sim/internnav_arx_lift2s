from .comm_utils.client import AgentClient

__all__ = ['AgentClient', 'AgentServer']


def __getattr__(name):
    # Lazy-load server so thin clients (robot) can import AgentClient without
    # pulling in full agent/server deps (habitat, quaternion, etc.).
    if name == 'AgentServer':
        from .comm_utils.server import AgentServer

        return AgentServer
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
