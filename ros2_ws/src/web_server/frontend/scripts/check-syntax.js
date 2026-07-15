import { execFileSync } from 'node:child_process';
import { existsSync, readFileSync, readdirSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';

function javascriptFiles(directory) {
    return readdirSync(directory, { withFileTypes: true }).flatMap((entry) => {
        const path = join(directory, entry.name);
        if (entry.isDirectory()) return javascriptFiles(path);
        return entry.isFile() && entry.name.endsWith('.js') ? [path] : [];
    });
}

javascriptFiles(new URL('../js', import.meta.url).pathname).forEach((file) => {
    execFileSync(process.execPath, ['--check', file], { stdio: 'inherit' });
    const source = readFileSync(file, 'utf8');
    for (const match of source.matchAll(/(?:from\s+|import\s*)['"](\.[^'"]+)['"]/g)) {
        const dependency = resolve(dirname(file), match[1]);
        if (!existsSync(dependency)) {
            throw new Error(`${file} imports missing module ${match[1]}`);
        }
    }
});
