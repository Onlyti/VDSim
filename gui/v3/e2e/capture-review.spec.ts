import { test, expect } from '@playwright/test'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const OUT = path.resolve(__dirname, '../../../docs/evidence/review/gui-captures')

type Mode = 'scenario' | 'run' | 'build' | 'analyze'

async function openMode(page: import('@playwright/test').Page, mode: Mode) {
  const setupReady = page.waitForResponse(
    (r) => r.url().includes('/api/setup') && r.request().method() === 'GET' && r.ok(),
    { timeout: 25_000 },
  )
  await page.goto(`index.html?m=${mode}#/${mode}`)
  await setupReady
  await expect(page.getByText(`Mode: ${mode}`)).toBeVisible({ timeout: 20_000 })
  await page.waitForTimeout(400)
}

test.describe('GUI capture review', () => {
  test('capture user flow', async ({ page }) => {
    test.setTimeout(120_000)
    fs.mkdirSync(OUT, { recursive: true })

    await openMode(page, 'scenario')
    const scenarioSel = page.locator('aside select.wide').first()
    if (await scenarioSel.count()) {
      const fleetLoad = page.waitForResponse(
        (r) => r.url().includes('/api/fleet') && r.request().method() === 'POST' && r.ok(),
        { timeout: 15_000 },
      )
      const setupRefresh = page.waitForResponse(
        (r) => r.url().includes('/api/setup') && r.request().method() === 'GET' && r.ok(),
        { timeout: 15_000 },
      )
      await scenarioSel.selectOption('l4_sedan_kinematics').catch(() => {})
      await fleetLoad.catch(() => {})
      await setupRefresh.catch(() => {})
      await expect(page.getByText(/\d+ points · figure8/)).toBeVisible({ timeout: 10_000 })
      await page.waitForTimeout(400)
    }
    await page.screenshot({ path: path.join(OUT, '01-scenario-l4.png'), fullPage: true })

    await openMode(page, 'build')
    await page.waitForTimeout(1200)
    await page.screenshot({ path: path.join(OUT, '02-build-default.png'), fullPage: true })

    const lvl = page.locator('select#lvl')
    if (await lvl.count()) {
      const asm = page.waitForResponse(
        (r) => r.url().includes('/api/catalog/assembly') && r.ok(),
        { timeout: 15_000 },
      )
      await lvl.selectOption('L4')
      await asm.catch(() => {})
      await page.waitForTimeout(1000)
    }
    const front = page.locator('button[data-slot="front_chassis"]')
    if (await front.count()) {
      await front.click()
      await page.waitForTimeout(2500)
    }
    await page.screenshot({ path: path.join(OUT, '03-build-l4-hardpoint.png'), fullPage: true })

    await openMode(page, 'analyze')
    await page.waitForTimeout(600)
    const picks = page.locator('label.pick input[type="checkbox"]')
    const n = await picks.count()
    for (let i = 0; i < Math.min(n, 2); i++) {
      if (!(await picks.nth(i).isChecked())) await picks.nth(i).check()
    }
    await page.screenshot({ path: path.join(OUT, '04-analyze-idle.png'), fullPage: true })
    const runBtn = page.getByRole('button', { name: 'Run compare' })
    if (await runBtn.isEnabled()) {
      const cmp = page.waitForResponse(
        (r) => r.url().includes('/api/compare') && r.request().method() === 'POST' && r.ok(),
        { timeout: 60_000 },
      )
      await runBtn.click()
      await cmp.catch(() => {})
      await page.waitForTimeout(1500)
      await page.screenshot({ path: path.join(OUT, '05-analyze-results.png'), fullPage: true })
    }

    await openMode(page, 'scenario')
    await expect(page.getByRole('button', { name: /Play/ })).toBeEnabled({ timeout: 15_000 })
    const setupPost = page.waitForResponse(
      (r) => r.url().includes('/api/setup') && r.request().method() === 'POST' && r.ok(),
      { timeout: 25_000 },
    )
    const startPost = page.waitForResponse(
      (r) =>
        (r.url().includes('/api/run/start') || r.url().includes('/api/control')) &&
        r.request().method() === 'POST' &&
        r.ok(),
      { timeout: 25_000 },
    )
    await page.getByRole('button', { name: /Play/ }).click()
    await setupPost
    await startPost
    await page.waitForTimeout(300)
    if (!(await page.getByText('Mode: run').isVisible().catch(() => false))) {
      await openMode(page, 'run')
    }
    await page.waitForTimeout(3000)
    await page.screenshot({ path: path.join(OUT, '06-run-sim-t3.png'), fullPage: true })
    await page.waitForTimeout(5000)
    await page.screenshot({ path: path.join(OUT, '07-run-sim-t8.png'), fullPage: true })
    await page.getByRole('button', { name: /Stop/ }).click().catch(() => {})
  })
})
