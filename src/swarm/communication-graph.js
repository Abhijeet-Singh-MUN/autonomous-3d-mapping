export class CommunicationGraph {
  constructor({ range, dropout = 0, latencyMs = 0, rng = Math.random }) {
    this.range = range;
    this.dropout = dropout;
    this.latencyMs = latencyMs;
    this.rng = rng;
    this.links = new Map();
  }

  update(agents) {
    this.links.clear();
    agents.forEach((agent) => this.links.set(agent.id, []));

    for (let a = 0; a < agents.length; a += 1) {
      for (let b = a + 1; b < agents.length; b += 1) {
        const first = agents[a];
        const second = agents[b];
        const distance = first.position.distanceTo(second.position);
        const dropped = this.dropout > 0 && this.rng() < this.dropout;
        if (distance <= this.range && !dropped) {
          this.links.get(first.id).push({ id: second.id, distance });
          this.links.get(second.id).push({ id: first.id, distance });
        }
      }
    }

    return this.snapshot();
  }

  neighborsOf(agentId) {
    return this.links.get(agentId) ?? [];
  }

  broadcast(senderId, payload, timeMs = 0) {
    return this.neighborsOf(senderId).map((neighbor) => ({
      from: senderId,
      to: neighbor.id,
      payload,
      deliverAtMs: timeMs + this.latencyMs,
      distance: neighbor.distance
    }));
  }

  connectedComponents() {
    const visited = new Set();
    const components = [];

    this.links.forEach((_, startId) => {
      if (visited.has(startId)) {
        return;
      }

      const component = [];
      const stack = [startId];
      visited.add(startId);

      while (stack.length) {
        const id = stack.pop();
        component.push(id);
        this.neighborsOf(id).forEach((neighbor) => {
          if (!visited.has(neighbor.id)) {
            visited.add(neighbor.id);
            stack.push(neighbor.id);
          }
        });
      }

      components.push(component);
    });

    return components;
  }

  snapshot() {
    const edges = [];
    this.links.forEach((neighbors, from) => {
      neighbors.forEach((neighbor) => {
        if (String(from) < String(neighbor.id)) {
          edges.push({ from, to: neighbor.id, distance: neighbor.distance });
        }
      });
    });

    return {
      edges,
      components: this.connectedComponents()
    };
  }
}
