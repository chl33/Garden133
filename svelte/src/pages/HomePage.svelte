<script>
  import { Thermometer, Droplets, Leaf, Battery, Sun, Activity, Zap, Info } from 'lucide-svelte';

  export let systemStatus;
  export let config;

  $: status = $systemStatus;
  $: conf = $config;

  function formatUptime(secs) {
    const days = Math.floor(secs / (24 * 3600));
    secs %= (24 * 3600);
    const hrs = Math.floor(secs / 3600);
    secs %= 3600;
    const mins = Math.floor(secs / 60);
    secs %= 60;
    
    let res = "";
    if (days > 0) res += `${days}d `;
    if (hrs > 0) res += `${hrs}h `;
    if (mins > 0) res += `${mins}m `;
    res += `${secs}s`;
    return res;
  }

  function safeToFixed(val, decimals) {
    if (val === undefined || val === null) return "0.0";
    return val.toFixed(decimals);
  }
</script>

<div class="dashboard">
  <header class="header">
    <h1>Garden133 Overview</h1>
    <div class="status-badge">
      <Info size={18} class="text-blue" />
      Boot Count: {status.bootCount}
    </div>
  </header>

  <div class="grid">
    <!-- Climate Status -->
    <section class="card">
      <div class="card-header">
        <Thermometer size={20} class="text-orange" />
        <h2>Climate</h2>
      </div>
      <div class="card-content">
        <div class="stat">
          <span class="label">Temperature</span>
          <span class="value">{safeToFixed(status.status.temperature, 1)}°C</span>
        </div>
        <div class="stat">
          <span class="label">Humidity</span>
          <span class="value">{safeToFixed(status.status.humidity, 1)}%</span>
        </div>
      </div>
    </section>

    <!-- Soil Status -->
    <section class="card">
      <div class="card-header">
        <Leaf size={20} class="text-green" />
        <h2>Soil Moisture</h2>
      </div>
      <div class="card-content">
        <div class="stat">
          <span class="label">Current</span>
          <span class="value">{safeToFixed(status.status.moisture, 1)}%</span>
        </div>
        <div class="stat">
          <span class="label">Filtered</span>
          <span class="value">{safeToFixed(status.status.moistureFilt, 1)}%</span>
        </div>
        <div class="stat">
          <span class="label">Raw counts</span>
          <span class="value">{status.status.moistureRaw}</span>
        </div>
      </div>
    </section>

    <!-- Power Status -->
    <section class="card">
      <div class="card-header">
        <Battery size={20} class="text-blue" />
        <h2>Power</h2>
      </div>
      <div class="card-content">
        <div class="stat">
          <span class="label">Battery Voltage</span>
          <span class="value">{safeToFixed(status.status.batteryVoltage, 2)}V</span>
        </div>
        <div class="stat">
          <span class="label">Solar Voltage</span>
          <span class="value">{safeToFixed(status.status.solarVoltage, 2)}V</span>
        </div>
        <div class="stat">
          <span class="label">5V Bus Voltage</span>
          <span class="value">{safeToFixed(status.status.fivevVoltage, 2)}V</span>
        </div>
        {#if status.status.charging !== undefined}
          <div class="stat">
            <span class="label">Charging</span>
            <span class="value" class:text-orange={status.status.charging}>
              {status.status.charging ? 'YES' : 'NO'}
            </span>
          </div>
          <div class="stat">
            <span class="label">Standby</span>
            <span class="value" class:text-green={status.status.standby}>
              {status.status.standby ? 'YES' : 'NO'}
            </span>
          </div>
        {/if}
      </div>
    </section>

    <!-- System Status -->
    <section class="card">
      <div class="card-header">
        <Activity size={20} />
        <h2>System</h2>
      </div>
      <div class="card-content">
        <div class="stat">
          <span class="label">Uptime</span>
          <span class="value">{formatUptime(status.uptime)}</span>
        </div>
        <div class="stat">
          <span class="label">Sleep Interval</span>
          <span class="value">{conf.sleepMin} min</span>
        </div>
        <div class="stat">
          <span class="label">MQTT</span>
          <span class="value" class:online={status.mqttConnected}>
            {status.mqttConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
      </div>
    </section>
  </div>
</div>

<style>
  .dashboard {
    display: flex;
    flex-direction: column;
    gap: 2rem;
  }

  .header {
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  h1 {
    font-size: 1.5rem;
    font-weight: 700;
    color: #111827;
  }

  .status-badge {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.5rem 1rem;
    background: white;
    border-radius: 9999px;
    box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
    font-size: 0.875rem;
    font-weight: 600;
  }

  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 1.5rem;
  }

  .card {
    background: white;
    border-radius: 0.75rem;
    padding: 1.5rem;
    box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  }

  .card-header {
    display: flex;
    align-items: center;
    gap: 0.75rem;
    margin-bottom: 1.25rem;
    border-bottom: 1px solid #f3f4f6;
    padding-bottom: 0.75rem;
  }

  .card-header h2 {
    font-size: 1.125rem;
    font-weight: 600;
    color: #374151;
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
    font-weight: 600;
    color: #111827;
  }

  .online {
    color: #059669;
  }

  :global(.text-green) { color: #10b981; }
  :global(.text-blue) { color: #3b82f6; }
  :global(.text-orange) { color: #f59e0b; }
</style>
