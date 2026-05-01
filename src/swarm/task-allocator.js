import { DEFAULT_SWARM_BEHAVIOR_PROFILE } from './behavior-profile.js';

export class TaskAllocator {
  constructor({ behaviorProfile = DEFAULT_SWARM_BEHAVIOR_PROFILE } = {}) {
    this.behaviorProfile = behaviorProfile;
  }

  assign({ agents, frontiers = [], aois = [], relayTargets = [] }) {
    const scoring = this.behaviorProfile.taskScoring;
    const tasks = [
      ...frontiers.map((frontier) => ({ ...frontier, type: 'frontier' })),
      ...aois.map((aoi) => ({ ...aoi, type: 'aoi' })),
      ...aois.map((aoi) => ({
        ...aoi,
        id: `${aoi.id}-verify`,
        type: 'verify',
        priority: (aoi.priority ?? 1) * scoring.verifyPriorityScale,
        informationGain: (aoi.informationGain ?? 1) * scoring.verifyInformationGainScale
      })),
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
      const scoring = this.behaviorProfile.taskScoring;
      const weight = scoring.roleWeights[agent.role]?.[task.type] ?? 1;
      const priority = task.priority ?? 1;
      const informationGain = task.informationGain ?? 0;
      const distance = task.position ? agent.position.distanceTo(task.position) : 0;
      const continuity = agent.currentGoal?.id === task.id ? scoring.continuityBias : 0;
      const communicationNeed = agent.behavior?.communicationHealth < scoring.weakCommunicationThreshold && task.type === 'relay'
        ? scoring.communicationRelayBonus
        : 0;
      const score = (
        priority * weight +
        informationGain * scoring.informationGainWeight +
        continuity +
        communicationNeed -
        distance * scoring.distancePenalty
      );

      if (score > bestScore) {
        best = task;
        bestScore = score;
      }
    });

    return best;
  }
}
