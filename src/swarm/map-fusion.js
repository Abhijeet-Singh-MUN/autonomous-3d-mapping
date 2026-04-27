export class MapFusion {
  constructor() {
    this.pointTiles = new Map();
    this.replayEvents = [];
  }

  ingestMapDeltas(agent) {
    const deltas = agent.localMapDeltas.splice(0);
    deltas.forEach((delta) => {
      this.replayEvents.push({
        type: 'map_delta',
        droneId: agent.id,
        time: delta.time ?? performance.now(),
        count: delta.count ?? 1
      });
    });
    return deltas.length;
  }

  ingestPointObservations(agent, tileSize = 4) {
    const observations = agent.pointObservations.splice(0);
    observations.forEach((observation) => {
      const point = observation.position;
      if (!point) {
        return;
      }
      const key = [
        Math.floor(point.x / tileSize),
        Math.floor(point.y / tileSize),
        Math.floor(point.z / tileSize)
      ].join(':');

      const tile = this.pointTiles.get(key) ?? {
        key,
        boundsHint: key.split(':').map(Number),
        contributors: new Set(),
        pointCount: 0,
        aoiTags: new Set()
      };

      tile.contributors.add(agent.id);
      tile.pointCount += observation.count ?? 1;
      (observation.aoiTags ?? []).forEach((tag) => tile.aoiTags.add(tag));
      this.pointTiles.set(key, tile);
    });

    return observations.length;
  }

  exportPointTiles() {
    return [...this.pointTiles.values()].map((tile) => ({
      key: tile.key,
      boundsHint: tile.boundsHint,
      contributors: [...tile.contributors],
      pointCount: tile.pointCount,
      aoiTags: [...tile.aoiTags]
    }));
  }

  recordReplayEvent(event) {
    this.replayEvents.push({
      time: performance.now(),
      ...event
    });
  }

  exportReplayLog() {
    return [...this.replayEvents];
  }
}
