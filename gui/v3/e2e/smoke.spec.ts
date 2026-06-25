import { test, expect } from '@playwright/test'

type Mode = 'scenario' | 'run' | 'build' | 'analyze'

async function openMode(page: import('@playwright/test').Page, mode: Mode) {
  const setupReady = page.waitForResponse(
    (r) => r.url().includes('/api/setup') && r.request().method() === 'GET' && r.ok(),
    { timeout: 25_000 },
  )
  await page.goto(`index.html#/${mode}`)
  await setupReady
  await expect(page.getByText('VDSim', { exact: true })).toBeVisible()
  await expect(page.getByText(`Mode: ${mode}`)).toBeVisible({ timeout: 15_000 })
}

test.describe('GUI v3 smoke', () => {
  test('scenario map and fleet', async ({ page }) => {
    await openMode(page, 'scenario')
    await expect(page.getByRole('button', { name: /Edit path/ })).toBeVisible()
    await expect(page.getByRole('button', { name: /Edit parts/ })).toBeVisible()
    await expect(page.getByRole('button', { name: 'Skidpad' })).toBeVisible()
    const tpl = page.waitForResponse(
      (r) => r.url().includes('/api/setup/template') && r.request().method() === 'POST' && r.ok(),
    )
    await page.locator('button.tpl').filter({ hasText: 'Figure-8' }).click()
    await tpl
    await expect(page.getByText(/\d+ points · figure8/)).toBeVisible()
  })

  test('build assembly', async ({ page }) => {
    await openMode(page, 'build')
    await expect(page.getByText('Vehicle build')).toBeVisible()
    await expect(page.getByRole('button', { name: 'V0' })).toBeVisible({ timeout: 15_000 })
    await expect(page.getByRole('button', { name: 'Export' })).toBeVisible()
  })

  test('analyze compare panel', async ({ page }) => {
    await openMode(page, 'analyze')
    await expect(page.getByText('ISO compare')).toBeVisible()
    await expect(page.getByRole('button', { name: 'Run compare' })).toBeVisible()
  })

  test('run viewport shell', async ({ page }) => {
    await openMode(page, 'run')
    await expect(page.getByRole('button', { name: 'Orbit' })).toBeVisible()
    await expect(page.getByRole('button', { name: 'Chase' })).toBeVisible()
    await expect(page.getByRole('button', { name: 'Top', exact: true })).toBeVisible()
    await expect(page.getByRole('button', { name: 'Autopilot' })).toBeVisible()
    await expect(page.getByRole('button', { name: 'Manual' })).toBeVisible()
  })
})
