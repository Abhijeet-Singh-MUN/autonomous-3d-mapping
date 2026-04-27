import { DRONE_ROLES } from './constants.js';

const ROLE_WEIGHTS = {
  [DRONE_ROLES.SCOUT]: { frontier: 1.4, aoi: 0.7, relay: 0.4 },
  [DRONE_ROLES.MAPPER]: { frontier: 0.9, aoi: 1.25, relay: 0.35 },
  [DRONE_ROLES.RELAY]: { frontier: 0.4, aoi: 0.4, relay: 1.5 },
  [DRONE_ROLES.VERIFIER]: { frontier: 0.55, aoi: 1.45, relay: 0.3 }
};

export class TaskAllocator {
  assign({ agents, frontiers = [], aois = [], relayTargets = [] }) {
    const tasks = [
      ...frontiers.map((frontier) => ({ ...frontier, type: 'frontier' })),
      ...aois.map((aoi) => ({ ...aoi, type: 'aoi' })),
      ...relayTargets.map((relay) => ({ ...relay, type: 'relay' }))
    ];
    const available = [...tasks];

    return agents.map((agent) => {
      const best = this.pickBestTask(agent, available);
      if (!best) {
        return { droneId: agent.id, role: agent.role, task: null };
      }
      available.splice(available.indexOf(best), 1);
      return {
        droneId: agent.id,
        role: agent.role,
        task: best
      };
    });
  }

  pickBestTask(agent, tasks) {
    let best = null;
    let bestScore = -Infinity;

    tasks.forEach((task) => {
      const weight = ROLE_WEIGHTS[agent.role]?.[task.type] ?? 1;
      const priority = task.priority ?? 1;
      const informationGain = task.informationGain ?? 0;
      const distance = task.position ? agent.position.distanceTo(task.position) : 0;
      const score = priority * weight + informationGain * 0.35 - distance * 0.08;

      if (score > bestScore) {
        best = task;
        bestScore = score;
      }
    });

    return best;
  }
}
