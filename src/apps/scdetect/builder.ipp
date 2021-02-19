#ifndef SCDETECT_APPS_SCDETECT_BUILDER_IPP_
#define SCDETECT_APPS_SCDETECT_BUILDER_IPP_

template <typename TProduct>
std::unique_ptr<TProduct> Builder<TProduct>::Build() {
  Finalize();
  return std::move(product_);
}

template <typename TProduct> void Builder<TProduct>::Finalize() {}

#endif // SCDETECT_APPS_SCDETECT_BUILDER_IPP_
