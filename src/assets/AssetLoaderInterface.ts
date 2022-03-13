export default interface AssetLoaderInterface {
    load(
        on_update: (progress: number, total: number) => void
    ): Promise<Map<string, any>>;

}