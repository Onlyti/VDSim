export type ScenarioTemplateId = 'empty' | 'figure8' | 'straight' | 'skidpad'

export interface ScenarioTemplate {
  id: ScenarioTemplateId
  label: string
  hint: string
}

export const SCENARIO_TEMPLATES: ScenarioTemplate[] = [
  { id: 'empty', label: 'Empty', hint: 'Flat road · short path · 1 vehicle' },
  { id: 'figure8', label: 'Figure-8', hint: 'Default track · v=10 m/s' },
  { id: 'straight', label: 'Straight', hint: '±40 m line · v=15 m/s' },
  { id: 'skidpad', label: 'Skidpad', hint: 'R=40 m circle · v=12 m/s' },
]
