<script>
  import { onMount } from 'svelte';
  import { Settings, Save, RefreshCcw } from 'lucide-svelte';

  export let config;

  $: conf = $config;

  let localConfig = {};

  function syncLocal() {
    localConfig = { ...$config };
  }

  onMount(() => {
    syncLocal();
  });

  async function saveConfig() {
    try {
      const response = await fetch('/api/config', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(localConfig)
      });
      if (response.ok) {
        alert('Configuration saved!');
        config.set({ ...localConfig });
      } else {
        alert('Failed to save configuration');
      }
    } catch (err) {
      console.error('Error saving config:', err);
      alert('Error saving configuration');
    }
  }

  function resetConfig() {
    syncLocal();
  }
</script>

<div class="config-page">
  <header class="header">
    <div class="title-with-icon">
      <Settings size={24} />
      <h1>Garden Control Configuration</h1>
    </div>
    <div class="actions">
      <button class="btn btn-secondary" on:click={resetConfig}>
        <RefreshCcw size={18} />
        Reset
      </button>
      <button class="btn btn-primary" on:click={saveConfig}>
        <Save size={18} />
        Save Configuration
      </button>
    </div>
  </header>

  <div class="config-grid">
    <!-- Operation Settings -->
    <section class="card">
      <h2>Operation Settings</h2>
      <div class="form-group">
        <label for="sleepMin">Sleep Interval (minutes)</label>
        <input id="sleepMin" type="number" step="1" bind:value={localConfig.sleepMin} />
        <p class="help">Time to deep sleep between sensor readings.</p>
      </div>
    </section>

    <!-- LoRa Settings -->
    <section class="card">
      <h2>LoRa Settings</h2>
      <div class="form-group">
        <label for="syncWord">Sync Word</label>
        <input id="syncWord" type="number" step="1" bind:value={localConfig.syncWord} />
        <p class="help">Must match your LoRa gateway (Default: 240/0xF0).</p>
      </div>
      <div class="form-group">
        <label for="frequency">Frequency</label>
        <select id="frequency" bind:value={localConfig.frequency}>
          <option value="433MHz">433 MHz</option>
          <option value="868MHz">868 MHz</option>
          <option value="915MHz">915 MHz</option>
        </select>
      </div>
      <div class="form-group">
        <label for="spreadingFactor">Spreading Factor (SF)</label>
        <select id="spreadingFactor" bind:value={localConfig.spreadingFactor}>
          <option value="SF7">SF7</option>
          <option value="SF8">SF8</option>
          <option value="SF9">SF9</option>
          <option value="SF10">SF10</option>
          <option value="SF11">SF11</option>
          <option value="SF12">SF12</option>
        </select>
        <p class="help">Higher means better range but slower transmission.</p>
      </div>
      <div class="form-group">
        <label for="signalBandwidth">Signal Bandwidth</label>
        <select id="signalBandwidth" bind:value={localConfig.signalBandwidth}>
          <option value="125k">125 kHz</option>
          <option value="500k">500 kHz</option>
        </select>
      </div>
    </section>

    <!-- Sensor Calibration -->
    <section class="card">
      <h2>Sensor Calibration</h2>
      <div class="form-group">
        <label for="moisture_in_min">Moisture 0% ADC counts</label>
        <input id="moisture_in_min" type="number" bind:value={localConfig.moisture_in_min} />
        <p class="help">ADC reading when sensor is dry.</p>
      </div>
      <div class="form-group">
        <label for="moisture_in_max">Moisture 100% ADC counts</label>
        <input id="moisture_in_max" type="number" bind:value={localConfig.moisture_in_max} />
        <p class="help">ADC reading when sensor is in water.</p>
      </div>
    </section>
  </div>
</div>

<style>
  .config-page {
    display: flex;
    flex-direction: column;
    gap: 2rem;
  }

  .header {
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .title-with-icon {
    display: flex;
    align-items: center;
    gap: 0.75rem;
  }

  h1 {
    font-size: 1.5rem;
    font-weight: 700;
    color: #111827;
  }

  .actions {
    display: flex;
    gap: 1rem;
  }

  .config-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
    gap: 1.5rem;
  }

  .card {
    background: white;
    border-radius: 0.75rem;
    padding: 1.5rem;
    box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  }

  h2 {
    font-size: 1.125rem;
    font-weight: 600;
    color: #374151;
    margin-bottom: 1.25rem;
    border-bottom: 1px solid #f3f4f6;
    padding-bottom: 0.75rem;
  }

  .form-group {
    margin-bottom: 1.25rem;
  }

  label {
    display: block;
    font-size: 0.875rem;
    font-weight: 500;
    color: #4b5563;
    margin-bottom: 0.375rem;
  }

  input, select {
    width: 100%;
    padding: 0.5rem;
    border: 1px solid #d1d5db;
    border-radius: 0.375rem;
    font-size: 0.875rem;
  }

  .help {
    font-size: 0.75rem;
    color: #6b7280;
    margin-top: 0.25rem;
  }

  .btn {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.5rem 1rem;
    border: none;
    border-radius: 0.375rem;
    font-weight: 600;
    cursor: pointer;
    font-size: 0.875rem;
  }

  .btn-primary {
    background: #059669;
    color: white;
  }

  .btn-primary:hover { background: #047857; }

  .btn-secondary {
    background: #f3f4f6;
    color: #374151;
  }

  .btn-secondary:hover { background: #e5e7eb; }
</style>
