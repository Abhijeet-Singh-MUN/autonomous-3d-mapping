export class CommunicationGraph {
  constructor({ range, maxNeighbors = 3, dropout = 0, latencyMs = 0, rng = Math.random }) {
    this.range = range;
    this.maxNeighbors = maxNeighbors;
    this.dropout = dropout;
    this.latencyMs = latencyMs;
    this.rng = rng;
    this.links = new Map();
  }

  update(agents) {
    this.links.clear();
    agents.forEach((agent) => this.links.set(agent.id, []));

    const candidates = [];
    for (let a = 0; a < agents.length; a += 1) {
      for (let b = a + 1; b < agents.length; b += 1) {
        const distance = agents[a].position.distanceTo(agents[b].position);
        if (distance <= this.range) {
          candidates.push({
            from: agents[a].id,
            to: agents[b].id,
            distance
          });
        }
      }
    }

    candidates
      .sort((a, b) => a.distance - b.distance)
      .forEach((edge) => {
        if (this.links.get(edge.from).length >= this.maxNeighbors || this.links.get(edge.to).length >= this.maxNeighbors) {
          return;
        }
        if (this.dropout > 0 && this.rng() < this.dropout) {
          return;
        }

        this.links.get(edge.from).push({ id: edge.to, distance: edge.distance });
        this.links.get(edge.to).push({ id: edge.from, distance: edge.distance });
      });

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
