<script>
  import { onMount } from 'svelte';
  import { Settings, Save, RefreshCcw, Droplets, Activity } from 'lucide-svelte';

  export let config;
  export let systemStatus;

  $: status = $systemStatus;

  let localConfig = {};

  function syncLocal() {
    localConfig = { ...$config };
  }

  onMount(() => {
    syncLocal();
  });

  async function saveConfig() {
    try {
      const response = await fetch('/api/moisture/config', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(localConfig)
      });
      if (response.ok) {
        alert('Calibration saved!');
        config.set({ ...localConfig });
      } else {
        alert('Failed to save calibration');
      }
    } catch (err) {
      console.error('Error saving config:', err);
      alert('Error saving calibration');
    }
  }

  function resetConfig() {
    syncLocal();
  }
</script>

<div class="config-page">
  <header class="header">
    <div class="title-with-icon">
      <Droplets size={24} />
      <h1>Moisture Sensor Calibration</h1>
    </div>
    <div class="actions">
      <button class="btn btn-secondary" on:click={resetConfig}>
        <RefreshCcw size={18} />
        Reset
      </button>
      <button class="btn btn-primary" on:click={saveConfig}>
        <Save size={18} />
        Save Calibration
      </button>
    </div>
  </header>

  <div class="config-grid">
    <!-- Live Readings -->
    <section class="card highlight">
      <div class="card-header">
        <Activity size={20} class="text-blue" />
        <h2>Live Readings</h2>
      </div>
      <div class="card-content">
        <div class="stat">
          <span class="label">Current ADC Counts</span>
          <span class="value">{status.status.moistureRaw}</span>
        </div>
        <div class="stat">
          <span class="label">Mapped Moisture</span>
          <span class="value">{status.status.moisture.toFixed(1)}%</span>
        </div>
        <div class="stat">
          <span class="label">Filtered Moisture</span>
          <span class="value">{status.status.moistureFilt.toFixed(1)}%</span>
        </div>
      </div>
      <p class="help mt-4">Use these live values to determine your 0% (dry) and 100% (wet) thresholds.</p>
    </section>

    <!-- Sensor Calibration -->
    <section class="card">
      <div class="card-header">
        <Settings size={20} />
        <h2>ADC Calibration</h2>
      </div>
      <div class="form-group">
        <label for="moistureInMin">Moisture 0% ADC counts</label>
        <input id="moistureInMin" type="number" bind:value={localConfig.moistureInMin} />
        <p class="help">ADC reading when sensor is dry.</p>
      </div>
      <div class="form-group">
        <label for="moistureInMax">Moisture 100% ADC counts</label>
        <input id="moistureInMax" type="number" bind:value={localConfig.moistureInMax} />
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

  .card.highlight {
    border: 2px solid #dbeafe;
    background: #f8fafc;
  }

  .card-header {
    display: flex;
    align-items: center;
    gap: 0.75rem;
    margin-bottom: 1.25rem;
    border-bottom: 1px solid #f3f4f6;
    padding-bottom: 0.75rem;
  }

  h2 {
    font-size: 1.125rem;
    font-weight: 600;
    color: #374151;
    margin: 0;
  }

  .stat {
    display: flex;
    justify-content: space-between;
    padding: 0.5rem 0;
  }

  .stat .label {
    color: #6b7280;
    font-size: 0.875rem;
  }

  .stat .value {
    font-weight: 700;
    color: #111827;
    font-family: monospace;
    font-size: 1.125rem;
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

  input {
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

  .mt-4 { margin-top: 1rem; }

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

  :global(.text-blue) { color: #3b82f6; }
</style>
