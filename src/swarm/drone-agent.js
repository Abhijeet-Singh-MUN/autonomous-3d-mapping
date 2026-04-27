import * as THREE from 'three';
import { DRONE_ROLES } from './constants.js';

export class DroneAgent {
  constructor({ id, role = DRONE_ROLES.SCOUT, position = new THREE.Vector3(), mesh = null }) {
    this.id = id;
    this.role = role;
    this.mesh = mesh;
    this.position = position.clone();
    this.velocity = new THREE.Vector3();
    this.assignment = null;
    this.currentGoal = null;
    this.routeSamples = [];
    this.routeSampleIndex = 0;
    this.mailbox = [];
    this.localMapDeltas = [];
    this.pointObservations = [];
    this.status = 'idle';
    this.metrics = {
      scans: 0,
      points: 0,
      replans: 0,
      distanceTraveled: 0
    };
  }

  setRole(role) {
    this.role = role;
  }

  setPosition(position) {
    this.position.copy(position);
    if (this.mesh) {
      this.mesh.position.copy(position);
    }
  }

  assign(assignment) {
    this.assignment = assignment;
    this.currentGoal = assignment?.goal ?? null;
    this.status = assignment ? 'assigned' : 'idle';
  }

  enqueueMessage(message) {
    this.mailbox.push(message);
  }

  drainMessages() {
    const messages = this.mailbox;
    this.mailbox = [];
    return messages;
  }

  recordMapDelta(delta) {
    this.localMapDeltas.push(delta);
  }

  recordPointObservation(observation) {
    this.pointObservations.push(observation);
    this.metrics.points += observation.count ?? 1;
  }

  snapshot() {
    return {
      id: this.id,
      role: this.role,
      status: this.status,
      position: this.position.toArray(),
      assignment: this.assignment,
      metrics: { ...this.metrics }
    };
  }
}
