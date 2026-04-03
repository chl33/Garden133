<script>
  import { Save } from 'lucide-svelte';

  // API base URL
  const API_BASE = '/api';

  export let mqtt;
  export let systemStatus;

  let mqttConfig;
  let sysStat;
  let saving = false;
  let saveMessage = '';

  $: mqttConfig = $mqtt;
  $: sysStat = $systemStatus;

  function updateMqtt(field, value) {
    mqtt.update(m => {
      m[field] = value;
      return m;
    });
  }

  async function saveMqttConfig() {
    saving = true;
    saveMessage = '';

    try {
      const response = await fetch(`${API_BASE}/mqtt`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(mqttConfig)
      });

      if (!response.ok) {
        throw new Error('Failed to save MQTT configuration');
      }

      saveMessage = 'MQTT configuration saved successfully!';
      setTimeout(() => saveMessage = '', 3000);
    } catch (err) {
      console.error('Error saving MQTT config:', err);
      saveMessage = `Error: ${err.message}`;
    } finally {
      saving = false;
    }
  }
</script>

<div class="page">
  <div class="header-row">
    <h2 class="page-title">MQTT Configuration</h2>
    <div class="status-badge" class:connected={sysStat.mqttConnected}>
      {sysStat.mqttConnected ? 'Connected' : 'Disconnected'}
    </div>
  </div>

  <div class="card">
    <div class="form-group checkbox-group">
      <label for="mqtt-enabled" class="checkbox-label">
        <input
          id="mqtt-enabled"
          type="checkbox"
          bind:checked={mqttConfig.enabled}
          on:change={() => updateMqtt('enabled', mqttConfig.enabled)}
        />
        Enable MQTT
      </label>
    </div>

    <div class="form-group">
      <label for="hostAddr" class="form-label">MQTT Broker Address</label>
      <input
        id="hostAddr"
        type="text"
        class="form-input"
        placeholder="e.g. 192.168.1.100 or broker.hivemq.com"
        bind:value={mqttConfig.hostAddr}
        on:change={() => updateMqtt('hostAddr', mqttConfig.hostAddr)}
      />
    </div>

    <div class="form-group">
      <label for="port" class="form-label">Port</label>
      <input
        id="port"
        type="number"
        class="form-input"
        bind:value={mqttConfig.port}
        on:change={() => updateMqtt('port', mqttConfig.port)}
      />
    </div>

    <div class="form-group">
      <label for="authUser" class="form-label">Username (optional)</label>
      <input
        id="authUser"
        type="text"
        class="form-input"
        bind:value={mqttConfig.authUser}
        on:change={() => updateMqtt('authUser', mqttConfig.authUser)}
      />
    </div>

    <div class="form-group">
      <label for="authPassword" class="form-label">Password (optional)</label>
      <input
        id="authPassword"
        type="password"
        class="form-input"
        bind:value={mqttConfig.authPassword}
        on:change={() => updateMqtt('authPassword', mqttConfig.authPassword)}
      />
    </div>

    <button class="btn btn-purple" on:click={saveMqttConfig} disabled={saving}>
      <Save size={20} />
      {saving ? 'Saving...' : 'Save MQTT Configuration'}
    </button>

    {#if saveMessage}
      <div class="save-message" class:error={saveMessage.startsWith('Error')}>
        {saveMessage}
      </div>
    {/if}
  </div>
</div>

<style>
  .header-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 1.5rem;
  }

  .page-title {
    font-size: 2rem;
    font-weight: 700;
    color: #1f2937;
    margin: 0;
  }

  .status-badge {
    padding: 0.4rem 1rem;
    border-radius: 2rem;
    background: #fee2e2;
    color: #991b1b;
    font-weight: 600;
    font-size: 0.875rem;
  }

  .status-badge.connected {
    background: #d1fae5;
    color: #065f46;
  }

  .card {
    background: white;
    padding: 1.5rem;
    border-radius: 0.5rem;
    box-shadow: 0 1px 3px rgba(0,0,0,0.1);
  }

  .form-group {
    margin-bottom: 1.5rem;
  }

  .form-label {
    display: block;
    font-size: 0.875rem;
    font-weight: 500;
    color: #374151;
    margin-bottom: 0.5rem;
  }

  .form-input {
    width: 100%;
    padding: 0.5rem 1rem;
    border: 1px solid #d1d5db;
    border-radius: 0.5rem;
    font-size: 1rem;
  }

  .form-input:focus {
    outline: none;
    border-color: #7c3aed;
    box-shadow: 0 0 0 3px rgba(124, 58, 237, 0.1);
  }

  .btn {
    display: inline-flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.75rem 1.5rem;
    border: none;
    border-radius: 0.5rem;
    font-size: 1rem;
    font-weight: 500;
    cursor: pointer;
    transition: background 0.2s;
  }

  .btn-purple {
    background: #7c3aed;
    color: white;
  }

  .btn-purple:hover {
    background: #6d28d9;
  }

  .btn-purple:disabled {
    background: #9ca3af;
    cursor: not-allowed;
  }

  .save-message {
    margin-top: 1rem;
    padding: 0.75rem 1rem;
    border-radius: 0.5rem;
    background: #dbeafe;
    color: #1e40af;
    font-size: 0.875rem;
    font-weight: 500;
  }

  .save-message.error {
    background: #fee2e2;
    color: #991b1b;
  }

  .checkbox-group {
    display: flex;
    align-items: center;
  }

  .checkbox-label {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    font-weight: 500;
    color: #374151;
    cursor: pointer;
  }

  input[type="checkbox"] {
    width: 1.25rem;
    height: 1.25rem;
    accent-color: #7c3aed;
  }
</style>
